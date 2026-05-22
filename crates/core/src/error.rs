//! Typed errors for the platform.
//!
//! Library crates return [`RoboticsError`] (a `thiserror` enum) so callers
//! can match on the variant. Binary crates (cli, examples) convert into
//! `anyhow::Error` at the application boundary.

use thiserror::Error;

pub type Result<T> = std::result::Result<T, RoboticsError>;

#[derive(Debug, Error)]
pub enum RoboticsError {
    #[error("joint {joint} angle {requested} rad out of limits [{min}, {max}]")]
    JointLimitViolation {
        joint: String,
        requested: f64,
        min: f64,
        max: f64,
    },

    #[error("target pose is outside the reachable workspace")]
    Unreachable,

    #[error("kinematics solver failed to converge: {0}")]
    KinematicsFailure(String),

    #[error("backend transport error: {0}")]
    Backend(String),

    #[error("hardware error: {0}")]
    Hardware(String),

    #[error("invalid configuration: {0}")]
    Config(String),

    #[error("invalid state transition from {from} to {to}")]
    InvalidStateTransition { from: String, to: String },

    #[error("emergency stop engaged")]
    EmergencyStop,

    #[error("I/O error")]
    Io(#[from] std::io::Error),
}
