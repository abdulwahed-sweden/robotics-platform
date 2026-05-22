//! The replay driver.
//!
//! Consumes a `ReplaySource` and dispatches every command against a
//! `Backend` with the original inter-command spacing (optionally scaled
//! by `speed`).

use std::time::Duration;

use anyhow::Result;
use chrono::{DateTime, Utc};
use robotics_core::{Backend, JointState, Vec3};
use robotics_kinematics::ArmModel;
use robotics_motion::MotionPlanner;
use robotics_protocols::Command;
use tokio::time::Instant;
use tracing::{info, warn};

use crate::source::ReplaySource;

#[derive(Clone, Copy, Debug)]
pub struct ReplayOptions {
    /// Playback rate. 1.0 = original timing. 10.0 = 10x faster.
    /// Must be strictly positive.
    pub speed: f64,
    /// Cap any inter-entry gap. Without this, a 4-hour idle pause in
    /// the log would block replay for 4 hours.
    pub max_gap: Duration,
    /// If true, an `EmergencyStop` in the log halts replay immediately
    /// (after dispatching). If false, replay continues.
    pub halt_on_estop: bool,
}

impl Default for ReplayOptions {
    fn default() -> Self {
        Self {
            speed: 1.0,
            max_gap: Duration::from_secs(5),
            halt_on_estop: true,
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct ReplayReport {
    pub observed: u64,
    pub applied: u64,
    pub rejected: u64,
    pub halted_on_estop: bool,
    pub wall_elapsed: Duration,
}

pub struct ReplayDriver {
    arm: ArmModel,
    opts: ReplayOptions,
}

impl ReplayDriver {
    pub fn new(arm: ArmModel, opts: ReplayOptions) -> Self {
        assert!(opts.speed > 0.0, "replay speed must be positive");
        Self { arm, opts }
    }

    /// Drive `backend` through every entry from `source`.
    pub async fn replay<S: ReplaySource>(
        &self,
        backend: &mut dyn Backend,
        mut source: S,
    ) -> Result<ReplayReport> {
        let mut report = ReplayReport::default();
        let mut prev_ts: Option<DateTime<Utc>> = None;
        let started = Instant::now();

        while let Some(entry) = source.next().await? {
            if let Some(prev) = prev_ts {
                let raw = entry
                    .t
                    .signed_duration_since(prev)
                    .to_std()
                    .unwrap_or(Duration::ZERO);
                let scaled =
                    Duration::from_secs_f64(raw.as_secs_f64() / self.opts.speed)
                        .min(self.opts.max_gap);
                if !scaled.is_zero() {
                    tokio::time::sleep(scaled).await;
                }
            }
            prev_ts = Some(entry.t);
            report.observed += 1;

            match self.dispatch(backend, &entry.cmd).await {
                Ok(()) => report.applied += 1,
                Err(e) => {
                    warn!(?entry.cmd, error = %e, "replay: dispatch failed");
                    report.rejected += 1;
                }
            }

            if self.opts.halt_on_estop && matches!(entry.cmd, Command::EmergencyStop) {
                info!("replay: e-stop in log — halting");
                report.halted_on_estop = true;
                break;
            }
        }

        report.wall_elapsed = started.elapsed();
        Ok(report)
    }

    async fn dispatch(
        &self,
        backend: &mut dyn Backend,
        cmd: &Command,
    ) -> robotics_core::Result<()> {
        match cmd {
            Command::EmergencyStop => backend.arm().emergency_stop().await,
            // FSM reset is observed by the state machine, not the backend.
            Command::Reset => Ok(()),
            Command::OpenGripper => backend.gripper().open().await,
            Command::CloseGripper => backend.gripper().close().await,
            Command::Home => backend.arm().apply_state(JointState::default()).await,
            Command::Move { target, approach_pitch } => {
                plan_and_apply(backend, &self.arm, *target, *approach_pitch).await
            }
            Command::Pick { target } | Command::Place { target } => {
                plan_and_apply(backend, &self.arm, *target, -std::f64::consts::FRAC_PI_2)
                    .await
            }
        }
    }
}

async fn plan_and_apply(
    backend: &mut dyn Backend,
    arm: &ArmModel,
    target: Vec3,
    approach: f64,
) -> robotics_core::Result<()> {
    let current = backend.arm().joint_state().await?;
    let planner = MotionPlanner::new(*arm);
    let traj = planner.plan_to_pose(current, target, approach, 0.5)?;
    backend.arm().apply_state(traj.end).await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::VecSource;
    use robotics_audit::AuditEntry;
    use robotics_protocols::Command;
    use std::time::Instant as StdInstant;

    /// Replay must not block on the recorded wall-clock gap when
    /// `speed` is very large — it should run nearly instantly even
    /// across days of gap between entries.
    #[tokio::test]
    async fn high_speed_collapses_gaps() {
        let now = Utc::now();
        let entries = vec![
            AuditEntry {
                t: now,
                op: "test".into(),
                cmd: Command::Home,
                out: "accepted".into(),
            },
            AuditEntry {
                t: now + chrono::Duration::hours(24),
                op: "test".into(),
                cmd: Command::Home,
                out: "accepted".into(),
            },
        ];
        let source = VecSource::new(entries);
        let driver = ReplayDriver::new(
            ArmModel::sg90_default(),
            ReplayOptions {
                speed: 1_000_000.0,
                max_gap: Duration::from_secs(2),
                halt_on_estop: true,
            },
        );

        let mut sim = robotics_simulation_stub();
        let started = StdInstant::now();
        let report = driver.replay(&mut sim, source).await.unwrap();

        assert_eq!(report.observed, 2);
        assert_eq!(report.applied, 2);
        assert!(
            started.elapsed() < Duration::from_secs(2),
            "replay should complete almost instantly under high speed, took {:?}",
            started.elapsed()
        );
    }

    /// Minimal `Backend` impl so the unit test stays inside this crate
    /// (no dependency on `robotics-simulation`). Just enough to satisfy
    /// the dispatch paths exercised by `Command::Home`.
    fn robotics_simulation_stub() -> StubBackend {
        StubBackend
    }

    struct StubBackend;

    #[async_trait::async_trait]
    impl robotics_core::Backend for StubBackend {
        fn name(&self) -> &'static str { "stub" }
        fn is_real(&self) -> bool { false }
        fn arm(&mut self) -> &mut dyn robotics_core::RobotArm { self }
        fn gripper(&mut self) -> &mut dyn robotics_core::Gripper { self }
        async fn start(&mut self) -> robotics_core::Result<()> { Ok(()) }
        async fn shutdown(&mut self) -> robotics_core::Result<()> { Ok(()) }
    }

    #[async_trait::async_trait]
    impl robotics_core::RobotArm for StubBackend {
        async fn joint_state(&self) -> robotics_core::Result<JointState> {
            Ok(JointState::default())
        }
        async fn apply(&mut self, _cmd: robotics_core::JointCommand) -> robotics_core::Result<()> {
            Ok(())
        }
        async fn emergency_stop(&mut self) -> robotics_core::Result<()> { Ok(()) }
    }

    #[async_trait::async_trait]
    impl robotics_core::Gripper for StubBackend {
        async fn open(&mut self) -> robotics_core::Result<()> { Ok(()) }
        async fn close(&mut self) -> robotics_core::Result<()> { Ok(()) }
        async fn state(&self) -> robotics_core::Result<robotics_core::GripperState> {
            Ok(robotics_core::GripperState::Open)
        }
    }
}
