# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Build / lint / test the whole workspace
cargo check --workspace                                  # fast cross-platform smoke check
cargo build --workspace
cargo clippy --workspace --all-targets -- -D warnings   # required clean before PR
cargo fmt --all                                          # required before pushing
cargo test --workspace                                   # required clean before PR

# Single crate / single test (real test names from the tree)
cargo test -p robotics-kinematics
cargo test -p robotics-kinematics ik_then_fk_roundtrip   # filter by name
cargo test -p robotics-planner happy_path_pick_and_place
cargo test -p robotics-kinematics -- --nocapture         # show println/tracing output

# Run the platform (the binary is `robotics`, in crate `robotics-cli`)
cargo run -p robotics-cli -- simulate                    # in-process pick-and-place
cargo run -p robotics-cli -- move --x 0.10 --y 0.0 --z 0.10
cargo run -p robotics-cli -- pick --x 0.12 --y 0.0 --z 0.05
cargo run --release -p robotics-cli -- hardware          # real PWM (Linux/ARM only)
cargo run -p robotics-cli -- hardware --dry-run          # stub PWM, logs commands (works on macOS)
cargo run --example render -p robotics-cli               # 3D viewer in the browser

# move / pick / place take an optional --hardware to target the hardware backend
# (otherwise they run against the in-process simulator)

# Structured logs are env-controlled
RUST_LOG=robotics_motion=trace,info cargo run -p robotics-cli -- simulate
```

CLI defaults expect configs at `configs/{arm,hardware,simulation}.toml`; override with `--arm`, `--hw`, `--sim`.

## Architecture — the load-bearing idea

There is one architectural rule and everything else follows from it:

**Every higher layer talks to `robotics_core::Backend` and nothing more concrete.** Both `SimulationBackend` and `HardwareBackend` implement it. The CLI picks one at runtime and stashes it in a `Box<dyn Backend>`; the planner, motion, and kinematics code cannot tell which one they got. If a change requires the planner (or anything above it) to know whether it's driving sim or hardware, the change is wrong — push the new capability through the trait surface in `crates/core` first.

### Layer order (strict — upper layers depend down, never up)

```text
cli           humans, config loading, backend selection — no robot logic
planner       tasks + state machine                     — no I/O, no motion math
motion        trajectories, easing                      — depends on kinematics, no backend
kinematics    pure sync FK/IK over &ArmModel            — no async, no I/O
core          traits + types only                       — no implementations
─── trait boundary ─────────────────────────────────────
simulation    kinematic sim (+ Gazebo bridge stub)
hardware      PWM-driven servos
gpio          Pi I/O (rppal on Linux, stub elsewhere)
```

Above the boundary: stateless math and orchestration, unit-testable on a laptop. Below: I/O and side effects.

`kinematics` is deliberately **sync** — the math has no I/O, and staying sync keeps it callable from ROS 2 callbacks or C FFI without an executor. Don't add `async` there.

`simulation` and `hardware` are siblings and share no code by design (sim-only assumptions must not leak into hardware). If something genuinely is shared, inline it until there are ≥3 consumers, then move it to `core`.

### Cross-platform builds: the gpio cfg-gate

`rppal` is Linux/ARM only. The `robotics-gpio` crate is the **only** place that touches it; it exposes `LinuxPwm` on Linux and `StubPwm` everywhere else behind the same `Pwm` trait. This is what lets `cargo check --workspace` and `cargo test --workspace` pass on macOS dev machines. Don't reach for `rppal` outside `robotics-gpio`.

### Determinism (don't break these)

The platform relies on four properties for record-and-replay and for tests that hold across refactors:

1. Sim tick is 200 Hz with `tokio::interval` + `MissedTickBehavior::Delay` — no busy-wait, no wall-clock dependence.
2. Trajectories are pure `sample(t)` functions of `(start, end, duration, easing)`.
3. The state machine in `planner` is fully explicit; bad transitions are rejected, not silently accepted.
4. IK is analytic (closed-form), not iterative — no convergence loop, no random seeds.

Adding wall-clock reads, iterative solvers, or silent state coercion breaks regression testing.

## Where new code goes

| You want to…                                         | Put it in…                                                                                            |
| ---------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| Add a joint / change geometry                        | `crates/kinematics` + `configs/arm.toml`                                                              |
| Add an easing / trajectory type                      | `crates/motion`                                                                                       |
| Add a new task or behavior                           | `crates/planner`                                                                                      |
| Wire a new backend (HAT, smart servo, Gazebo bridge) | new module in `crates/hardware` or `crates/simulation` implementing `Backend` from `crates/core`      |
| Add a vision detector                                | `crates/vision`                                                                                       |
| Add a CLI subcommand                                 | `crates/cli`                                                                                          |

If you find yourself reaching across crates, the boundary is probably wrong — stop and reconsider before adding the dependency.

Adding a joint: extend `JointId` and the corresponding rows in `JointState` / `ArmModel` — the compiler will then enumerate every site that needs updating.

## Safety contract (don't bypass)

Every command passes through three independent limit checks before driving a motor: kinematic limits in IK, backend limits in `apply`, and `ServoCalibration` limits on the hardware side. They are intentionally not equal — kinematic limit ⊂ hardware calibration limit ⊂ mechanical limit, each with a ~2° margin. **Do not collapse them to one check** to "DRY it up"; that margin absorbs floating-point trajectory overshoot and is what stops gear damage.

`RobotArm::emergency_stop()` is part of the trait (not a backend extra) and must remain idempotent and infallible from the caller's perspective. The `Estop` transition is universally available from every state in the FSM — there is no "I can't stop right now" path.

PRs that touch limits, the e-stop path, or PWM disable should be tagged `safety` in the description; see `docs/safety.md` for the full contract.

## Conventions

- Use `tracing` for logs, not `println!`. The control loop is the product; structured logs are how it's debugged.
- Public items get at least a one-line doc comment.
- No `unsafe` without a comment explaining why.
- New dependencies need a one-paragraph justification in the PR.
- Workspace deps are centralized in the root `Cargo.toml`; crates pull versions with `{ workspace = true }`. Add new shared deps there, not per-crate.

## Further reading

`docs/architecture.md` (the "why"), `docs/kinematics.md` (the math), `docs/safety.md` (the contract), `docs/hardware.md` (Pi bring-up + calibration), `docs/simulation.md` (in-process sim + Gazebo bridge options).
