# Ambitions

The long-form north-star document for `robotics-platform`. [`roadmap.md`](roadmap.md) covers what gets built next; this document covers what the project **could become** in two-to-five years, and which existing Rust projects we'd stand on the shoulders of to get there.

The premise: robotics is the field that needs Rust the most. Deterministic timing, no GC pauses, ownership rules that catch the entire C++ category of memory bugs, an async runtime as good as anyone has, and a real `no_std` story for the embedded edge. The Rust robotics ecosystem has reached the inflection point where you can build a production stack end-to-end — controller, simulation, vision, learning — without leaving the language. This document is the case for doing exactly that.

---

## Tier 1 — Architectural Evolutions

Each of these changes what the platform fundamentally **is**, not just what it can do. Pick one as the focus of the next minor release.

### 1. Full dynamic simulation with [Rapier](https://rapier.rs/)

Replace the kinematic simulator with a real rigid-body physics engine. Rapier is pure Rust, parallel, deterministic, and used in production by several robotics teams.

What this unlocks:
- **Contact physics.** Grasping is no longer "snap to object if close enough" — you simulate the actual friction cone at the gripper-cube contact.
- **Mass and inertia.** Trajectories that work in kinematic sim but oscillate under real loads now show up here.
- **Multi-body chains.** The same engine handles a 5-DOF arm and a 24-DOF humanoid hand.
- **Determinism.** Rapier is deterministic across platforms with the `enhanced-determinism` feature — record-and-replay works for free.

Implementation surface: a `RapierBackend` next to `SimulationBackend`, both implementing `Backend`. The trait surface doesn't move.

Bonus: pair with [`parry`](https://parry.rs/) (Rapier's collision library) for **swept-volume collision checking** during motion planning — the arm refuses to plan through itself.

### 2. Native 3D viewer with [Bevy](https://bevyengine.org/) + [wgpu](https://wgpu.rs/)

The browser viewer is great for demos. A native ECS-based editor is great for development.

A Bevy plugin that:
- Renders the arm live, driven by the same telemetry that feeds the browser viewer.
- Lets you scrub time, inject e-stops, drag IK targets in 3D with `bevy_mod_picking`.
- Hosts an `egui` panel with joint sliders, config reload, trajectory plots.
- Targets desktop natively AND compiles to WebAssembly via `wgpu` — same code, two deployment surfaces.

Bevy's ECS also makes it natural to express the world: every joint is an entity, every sensor is an entity, every cube is an entity, and the kinematic chain falls out of parent-child transforms.

### 3. [Rerun](https://rerun.io/) integration for time-travel debugging

[`rerun`](https://github.com/rerun-io/rerun) is purpose-built for visualizing robotics/CV data over time. It's Rust-native and the team behind it ships a logging crate `rerun` that you call from your Rust code:

```rust
rec.log("robot/joints", &joints)?;
rec.log("robot/end_effector", &Transform3D::from(...))?;
rec.log("camera/rgb", &Image::from_rgb(...))?;
```

Then `rerun` opens a time-scrubbable timeline with **every log call across every entity** synchronized. Bugs in robot motion that take hours to track down in `println!` traces show up in two minutes here.

Integration: one feature flag (`rerun-telemetry`), one telemetry adapter, every joint state and every detection auto-logged. Free observability superpower.

### 4. ROS 2 native bridge via [`r2r`](https://github.com/sequenceplanner/r2r) or [`rclrs`](https://github.com/ros2-rust/ros2_rust)

The roadmap mentions this as the "production path" for Gazebo. Doing it properly means:
- `JointState` publisher on `/joint_states`.
- `JointTrajectory` subscriber on `/<robot>/joint_trajectory`.
- `tf2` broadcaster for the full kinematic chain.
- `lifecycle` node integration so the platform plays nicely with launch and rosbag.

Once this lands, the platform inherits the entire ROS 2 ecosystem — MoveIt 2 for motion planning, Nav2 for mobile bases, the entire perception stack.

### 5. WebAssembly plugin system via [`wasmtime`](https://wasmtime.dev/)

The most interesting architectural play. Let users write **behavior policies** in any WASM-targeting language and load them as sandboxed plugins:

```rust
let policy = wasmtime::Module::from_file(&engine, "pick_strategy.wasm")?;
let outcome = backend.execute_with_policy(&policy, &task)?;
```

Use cases:
- **Hot reload** during development — swap policies without restart.
- **Sandboxing** — policies can't take down the control loop. WASM has no syscall surface by default.
- **Multi-tenant** — a robotics-as-a-service platform where each tenant ships their own policy module.
- **AI integration** — an LLM writes a WASM strategy on the fly and you load it.

### 6. Real-time edge via [Embassy](https://embassy.dev/) on microcontrollers

Move the safety-critical low-level loop (PWM heartbeat, e-stop monitoring, limit interlocks) off the Pi entirely and onto a **dedicated microcontroller** running Embassy.

Why: a Pi running Linux is *not* a real-time system. Cosmic-ray-class jitter on the PWM line can twitch a servo. An RP2040 or STM32G4 running Embassy gives you nanosecond-precise PWM, hardware interrupt response to the e-stop button, and zero possibility of GC or scheduler pauses.

The Pi becomes the planner; the microcontroller is the safety layer. They speak over UART or CAN. Same Rust language, drastically different timing guarantees.

### 7. Vision pipeline with [tract](https://github.com/sonos/tract) or [burn](https://burn.dev/) + [realsense-rust](https://github.com/Tangram-Vision/realsense-rust)

A full Rust vision stack:
- **[`realsense-rust`](https://github.com/Tangram-Vision/realsense-rust)** — Intel RealSense depth cameras with proper bindings.
- **[`tract`](https://github.com/sonos/tract)** — ONNX inference. YOLO, OWL-ViT, SAM, all the modern open-vocab detectors. Pure Rust, no Python, no ONNX Runtime.
- **[`burn`](https://burn.dev/)** — newer, supports training as well as inference. WGPU backend means it runs on the same GPU as the viewer.
- **[`opencv-rust`](https://github.com/twistedfall/opencv-rust)** — for the classical CV pre/post-processing pieces that aren't worth re-implementing.

A `CameraDetector` impl of the existing `ObjectDetector` trait does the work. Calibration tooling becomes a CLI subcommand (`robotics calibrate camera --intrinsics …`).

---

## Tier 2 — Major Features

### Reinforcement learning training loop

Train manipulation policies *in Rust*, end-to-end. The stack:

- **[`burn`](https://burn.dev/)** or **[`candle`](https://github.com/huggingface/candle)** for the policy network.
- **The kinematic + rapier simulator** for the environment (millions of episodes/s on a beefy box with parallel envs).
- **[`tch-rs`](https://github.com/LaurentMazare/tch-rs)** if you need PyTorch interop for the algorithm itself.

The full RL inner loop — observation → policy → action → step → reward → backprop — staying inside Rust means no Python serialization tax. The same trained policy serializes to ONNX and loads via `tract` for production inference.

### Multi-robot coordination via [Zenoh](https://zenoh.io/)

[Zenoh](https://github.com/eclipse-zenoh/zenoh) is a pub/sub protocol designed by the same people behind DDS but with a much simpler API and better edge story. It's pure Rust on the wire side and has bridges to ROS 2.

What it unlocks: a coordinator process subscribes to every arm's `/state`, publishes `/goals/<arm-id>`. Each arm is a separate `Backend` instance. The same `RobotArm` trait that drove one arm now scales to a fleet without any new architecture.

Bonus: [`iceoryx2`](https://github.com/eclipse-iceoryx/iceoryx2) for zero-copy IPC between processes on the same machine. Sub-microsecond message passing for tight control loops.

### Type-safe SI units with [`uom`](https://github.com/iliekturtles/uom)

The platform currently encodes radians, meters, and seconds in `f64` and trusts the human to keep them straight. [`uom`](https://crates.io/crates/uom) gives compile-time units:

```rust
let angle: Angle<f64> = 1.5 * radian;
let length: Length<f64> = 0.10 * meter;
let v = length / time;   // compile error if you forgot the time
```

Apply to `JointLimits`, `JointCommand`, the whole kinematics surface. Every unit confusion bug becomes a compile error.

### Formal safety verification with [Kani](https://github.com/model-checking/kani)

[Kani](https://model-checking.github.io/kani/) is a Rust model checker from AWS. It proves properties of your code with bounded symbolic execution.

What we'd prove:
- `JointLimits::check` never accepts an out-of-range angle.
- The state machine's `transition` function rejects every disallowed pair.
- `ServoCalibration::angle_to_duty` never returns a duty outside `[0.0, 1.0]`.
- `HardwareBackend::emergency_stop` always disables every channel, regardless of initial state.

These are exactly the safety-critical surfaces called out in [`safety.md`](safety.md). Kani proofs go in CI; a failed proof blocks the merge. This is how you move toward IEC 61508 compliance without the proprietary toolchain.

### Behavior trees with [`bonsai-bt`](https://github.com/Sollimann/bonsai) or scripted via [`mlua`](https://github.com/khvzak/mlua)

The procedural pick-and-place is fine for v0.1; for anything branchier (retry on failure, parallel sensor-checked actions, hierarchical task decomposition) a behavior tree is the right abstraction.

[`bonsai-bt`](https://github.com/Sollimann/bonsai) is a pure-Rust BT. [`mlua`](https://github.com/khvzak/mlua) lets non-Rust users author behaviors in Lua. The choice is a strict-typing/easy-authoring tradeoff and we'd ship both.

### Industrial integration

- **OPC UA** server via [`opcua`](https://github.com/locka99/opcua) — the lingua franca of factory automation.
- **EtherCAT** — via [`ethercrab`](https://github.com/ethercrab-rs/ethercrab), a pure-Rust EtherCAT master. Drive industrial servos at 1 kHz.
- **MQTT** for IoT-class telemetry via [`rumqttc`](https://github.com/bytebeamio/rumqtt).
- **MAVLink** via [`mavlink-rust`](https://github.com/mavlink/rust-mavlink) — if a drone arm makes sense for the project.

### Distributed sensor data lake with [`arrow-rs`](https://github.com/apache/arrow-rs) + [`datafusion`](https://github.com/apache/datafusion)

Every joint state, every sensor reading, every command — Arrow columnar format, Parquet on disk, queryable with SQL via DataFusion. This is the foundation for offline analysis ("show me every trajectory that hit a joint limit in March"), regression testing ("does today's controller perform within 2σ of last week's?"), and ML data pipelines.

---

## Tier 3 — Research Moonshots

These are research-program-scale. Each could be its own PhD.

### Differentiable simulation for sim-to-real

Replace Rapier with [`brax`-style](https://github.com/google/brax) **differentiable physics**. Lets you backpropagate through the entire trajectory — gradient-based controller design, analytic sim-to-real adaptation. There's no fully-baked Rust differentiable physics engine today; this is genuinely greenfield. Candle + custom contact solver is the path.

### LLM-as-controller

Expose the platform as a **tool** an LLM can call. The model receives a scene description + goal, emits a sequence of `PickPlaceTask` invocations. Anthropic's Claude or any tool-use-capable model. The MCP (Model Context Protocol) maps cleanly: every robot capability becomes an MCP tool, the LLM orchestrates them.

The interesting research question: **constraint propagation**. The LLM has to know what's reachable; we'd need to expose IK feasibility as a tool the model can probe before committing.

### Neural inverse kinematics

For a 5-DOF arm the analytic solver is correct and constant-time, no contest. For a 7+ DOF redundant arm, analytic doesn't exist and iterative solvers are slow.

A neural IK model — trained on millions of FK samples from the kinematic simulator — gives sub-microsecond inference and lets the redundancy resolution be **learned** rather than hand-engineered. [`burn`](https://burn.dev/) trains it; [`tract`](https://github.com/sonos/tract) runs it. The trait surface (`solve(target) -> JointState`) doesn't change.

### Federated learning across a fleet

A hundred arms doing pick-and-place every day. Each one learns from its local successes and failures. **Federated learning** (each arm trains a local gradient, only the gradient is uploaded) means the fleet improves without anyone uploading raw video. Privacy-preserving by construction.

[`Linfa`](https://github.com/rust-ml/linfa) for the classical ML, burn for the neural pieces. Federated averaging is ~50 lines of code on top of zenoh.

### Apple Vision Pro / AR tele-op via [`openxr-rs`](https://github.com/Ralith/openxrs)

Wear the headset. See the arm as a hologram overlaid on the real arm. Reach out with your hand. The arm mirrors your motion in real time via inverse kinematics. Pinch to close the gripper.

`openxr-rs` is Rust bindings to the cross-platform OpenXR standard — works on Vision Pro, Quest, Index, Pico. The gesture tracking flows into the same `MotionPlanner` we already have.

### Formal verification of the entire planner

Beyond Kani's bounded model checking — full functional verification with [`Prusti`](https://github.com/viperproject/prusti-dev) or [`Creusot`](https://github.com/creusot-rs/creusot). Prove that the motion planner *never* emits a trajectory that violates joint limits, *never* exceeds velocity ceilings, *never* deadlocks. The kind of guarantee currently reserved for nuclear and aerospace control software, becoming approachable for hobby robotics.

---

## The Rust robotics ecosystem we'd stand on

| Crate / project | What we'd use it for |
|---|---|
| [Rapier](https://rapier.rs/) | Full rigid-body physics simulation |
| [Parry](https://parry.rs/) | Collision detection for motion planning |
| [Bevy](https://bevyengine.org/) | Native 3D editor / viewer |
| [wgpu](https://wgpu.rs/) | Cross-platform GPU (desktop + WASM) |
| [egui](https://github.com/emilk/egui) | Native UI panels |
| [Rerun](https://rerun.io/) | Time-travel telemetry visualization |
| [k](https://github.com/openrr/k) | Alternative URDF-driven IK |
| [r2r](https://github.com/sequenceplanner/r2r) / [rclrs](https://github.com/ros2-rust/ros2_rust) | ROS 2 bindings |
| [wasmtime](https://wasmtime.dev/) | Sandboxed plugin system for policies |
| [Embassy](https://embassy.dev/) | Async embedded for microcontroller edge |
| [RTIC](https://rtic.rs/) | Hard real-time framework, alternative to Embassy |
| [tract](https://github.com/sonos/tract) | ONNX inference for vision and IK |
| [burn](https://burn.dev/) | ML training + inference, WGPU backend |
| [candle](https://github.com/huggingface/candle) | Minimalist ML, transformer-friendly |
| [opencv-rust](https://github.com/twistedfall/opencv-rust) | OpenCV bindings for classical CV |
| [realsense-rust](https://github.com/Tangram-Vision/realsense-rust) | Intel RealSense depth cameras |
| [Zenoh](https://zenoh.io/) | Multi-robot pub/sub |
| [iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) | Zero-copy IPC |
| [uom](https://github.com/iliekturtles/uom) | Compile-time SI units |
| [Kani](https://github.com/model-checking/kani) | Bounded model checking |
| [Prusti](https://github.com/viperproject/prusti-dev) | Functional verification |
| [Creusot](https://github.com/creusot-rs/creusot) | Verification with deductive proofs |
| [bonsai-bt](https://github.com/Sollimann/bonsai) | Behavior trees |
| [mlua](https://github.com/khvzak/mlua) | Lua scripting for behavior |
| [opcua](https://github.com/locka99/opcua) | OPC UA industrial protocol |
| [ethercrab](https://github.com/ethercrab-rs/ethercrab) | EtherCAT master in pure Rust |
| [rumqttc](https://github.com/bytebeamio/rumqtt) | MQTT client |
| [arrow-rs](https://github.com/apache/arrow-rs) | Columnar data format |
| [datafusion](https://github.com/apache/datafusion) | SQL over robot logs |
| [tokio-console](https://github.com/tokio-rs/console) | Async runtime debugger |
| [openxr-rs](https://github.com/Ralith/openxrs) | VR/AR tele-op |
| [Linfa](https://github.com/rust-ml/linfa) | Classical ML, federated learning building blocks |

---

## Why Rust specifically wins for robotics

Most robotics stacks pick two of these and live with the third:

| Property | C++ | Python | **Rust** |
|---|---|---|---|
| Deterministic latency | ✓ | ✗ | ✓ |
| Memory safety | ✗ | ✓ | ✓ |
| Async without runtime headaches | △ | △ | ✓ |
| No GC pauses | ✓ | ✗ | ✓ |
| `no_std` / embedded | ✓ | ✗ | ✓ |
| Modern type system (sum types, traits) | △ | ✗ | ✓ |
| Single language from MCU to cloud | ✗ | ✗ | ✓ |

ROS 2 is "C++ for the loop, Python for the high level." Rust collapses that into one language with neither tradeoff. **A team that picks Rust today is buying back 30% of its future maintenance burden.**

---

## What this becomes

If even half of Tier 1 lands, the project stops being a hobby foundation and becomes a credible **open-source Rust robotics OS**: dynamic simulation, native + browser visualization, time-travel debugging, ROS 2 interoperability, sandboxed policy plugins, and a real-time edge — all in one language, one workspace.

The benchmark to clear: a team can take this platform off the shelf and ship a research arm in a weekend, a hobbyist arm in a week, an industrial cell in a quarter. Anything that helps that scenario is worth doing; everything else is decoration.

The motion code you wrote against `Backend` on day one keeps running.
