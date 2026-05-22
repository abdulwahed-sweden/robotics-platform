//! `clap`-derived argument tree.

use clap::{Parser, Subcommand};

#[derive(Debug, Parser)]
#[command(
    name = "robotics",
    version,
    about = "Rust robotics platform CLI",
    long_about = None,
)]
pub struct Cli {
    #[arg(long, default_value = "configs/arm.toml")]
    pub arm: String,

    #[arg(long, default_value = "configs/hardware.toml")]
    pub hw: String,

    #[arg(long, default_value = "configs/simulation.toml")]
    pub sim: String,

    #[command(subcommand)]
    pub cmd: Command,
}

#[derive(Debug, Subcommand)]
pub enum Command {
    /// Run the in-process simulator with the pick-and-place demo.
    Simulate,
    /// Drive real hardware. On non-Linux, uses the stub PWM (no
    /// physical movement, but commands are logged for inspection).
    Hardware {
        /// Don't actually drive PWM; print what would have happened.
        /// (Equivalent to non-Linux fallback, but explicit.)
        #[arg(long)]
        dry_run: bool,
    },
    /// Calibration helpers (run interactively).
    Calibrate {
        #[command(subcommand)]
        sub: CalibrateCmd,
    },
    /// Move the end-effector to (x, y, z) and stop.
    Move {
        #[arg(long, allow_hyphen_values = true)]
        x: f64,
        #[arg(long, allow_hyphen_values = true)]
        y: f64,
        #[arg(long, allow_hyphen_values = true)]
        z: f64,
        /// Approach pitch in radians. Default is straight down.
        #[arg(long, allow_hyphen_values = true, default_value_t = -std::f64::consts::FRAC_PI_2)]
        approach: f64,
        /// Use hardware backend instead of simulation.
        #[arg(long)]
        hardware: bool,
    },
    /// Pick the object at (x, y, z).
    Pick {
        #[arg(long, allow_hyphen_values = true)]
        x: f64,
        #[arg(long, allow_hyphen_values = true)]
        y: f64,
        #[arg(long, allow_hyphen_values = true)]
        z: f64,
        #[arg(long)]
        hardware: bool,
    },
    /// Place the held object at (x, y, z).
    Place {
        #[arg(long, allow_hyphen_values = true)]
        x: f64,
        #[arg(long, allow_hyphen_values = true)]
        y: f64,
        #[arg(long, allow_hyphen_values = true)]
        z: f64,
        #[arg(long)]
        hardware: bool,
    },
    /// Replay a recorded audit log (JSONL) against a fresh simulator.
    Replay {
        /// Path to the audit log file produced by the dashboard.
        #[arg(long)]
        from: std::path::PathBuf,
        /// Playback rate. 1.0 = original timing, 10.0 = 10x faster.
        #[arg(long, default_value_t = 1.0)]
        speed: f64,
    },
}

#[derive(Debug, Subcommand)]
pub enum CalibrateCmd {
    /// Drive every joint to its midpoint and hold.
    Center,
    /// Sweep one joint min↔max so you can read endstops.
    Sweep {
        #[arg(long)]
        joint: String,
    },
}
