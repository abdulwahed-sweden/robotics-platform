//! `robotics` — the platform's CLI entry point.
//!
//! ```text
//! USAGE:
//!   robotics simulate                            # in-process sim
//!   robotics hardware                            # real arm
//!   robotics calibrate sweep --joint shoulder
//!   robotics move --x 0.10 --y 0.0 --z 0.10
//!   robotics pick --x 0.10 --y 0.0 --z 0.05
//!   robotics place --x 0.10 --y 0.10 --z 0.05
//! ```
//!
//! Global flags:
//!
//!   --arm    path to arm.toml         (default: configs/arm.toml)
//!   --hw     path to hardware.toml    (default: configs/hardware.toml)
//!   --sim    path to simulation.toml  (default: configs/simulation.toml)

mod args;
mod config;
mod run;

use anyhow::Result;
use clap::Parser;
use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() -> Result<()> {
    // Structured logs, env-controlled level. `RUST_LOG=robotics_motion=trace`
    // gives you the planner's IK output, etc.
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .with_target(true)
        .compact()
        .init();

    let cli = args::Cli::parse();
    run::dispatch(cli).await
}
