# Contributing

Thanks for considering a contribution. Keep things small, focused,
and tested and they will move fast.

## Setup

```bash
git clone https://github.com/abdulwahed-sweden/robotics-platform
cd robotics-platform
cargo test --workspace
cargo run -p robotics-cli -- simulate
```

You need Rust stable. On Linux/ARM the `hardware` subcommand drives
real PWM via `rppal`; everywhere else it falls back to a stub so the
workspace builds and tests run.

## Where to put new code

Read [docs/architecture.md](docs/architecture.md) first — the
crate boundaries are load-bearing.

| You want to… | Put it in… |
|--------------|------------|
| Add a joint / change geometry  | `crates/kinematics` + `configs/arm.toml` |
| Add an easing / trajectory type | `crates/motion` |
| Add a new task or behavior      | `crates/planner` |
| Wire a new backend (HAT, smart servo, Gazebo bridge) | new module in `crates/hardware` or `crates/simulation`, implementing the `Backend` trait from `crates/core` |
| Add a vision detector           | `crates/vision` |
| Add a CLI subcommand            | `crates/cli` |

If you find yourself reaching across crates, the boundary is probably
wrong — open an issue first.

## Style

```bash
cargo fmt --all          # required before pushing
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

- Public items have at least a one-line doc comment.
- Use `tracing` for logs, not `println!`. The control loop is the
  product, structured logs are how you debug it.
- No `unsafe` without a comment explaining why.
- No new dependencies without a one-paragraph justification in the PR.

## Commits

- Small. One concern per commit.
- Imperative subject line: "Add X", "Fix Y", not "Added X".
- Body explains *why*, not what (the diff already shows what).
- Don't bundle formatting changes with logic changes.

## Pull requests

- All tests pass (`cargo test --workspace`).
- `cargo clippy -- -D warnings` is clean.
- New behavior has at least one test.
- Doc updates land in the same PR as the code change they describe.
- If you touched a public trait, mention which existing impls were
  updated.

## Safety changes

Anything that touches limits, e-stop, or PWM disable paths is a
safety change. Tag the PR `safety` in the description; reviewers will
look harder. See [docs/safety.md](docs/safety.md) for the contract.

## License

By contributing, you agree your work is dual-licensed under
[MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE), matching the
project.
