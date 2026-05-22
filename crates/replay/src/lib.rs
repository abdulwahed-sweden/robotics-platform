//! # robotics-replay
//!
//! Reads `AuditEntry` records and re-dispatches each command against
//! a `Backend`. The same code path that drove the original run drives
//! the replay — IK, motion planner, FSM-aware sequencing — so the
//! determinism contract from `docs/architecture.md` reduces to
//!
//!   FK( apply( replay( audit( run ) ) ) )  ≈  FK( apply( run ) )
//!
//! within numerical tolerance.

mod driver;
mod source;

pub use driver::{ReplayDriver, ReplayOptions, ReplayReport};
pub use source::{JsonlSource, ReplaySource, VecSource};
