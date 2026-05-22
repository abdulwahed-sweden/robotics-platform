//! Run the dashboard against the in-process simulator with a demo
//! motion loop driving the arm so visitors see something move.
//!
//! ```bash
//! cargo run -p robotics-dashboard --example dashboard_sim
//! # then open http://localhost:8080
//! ```

use std::time::Duration;

use anyhow::Result;
use robotics_audit::AuditRecorder;
use robotics_core::{Backend, Vec3};
use robotics_dashboard::{Dashboard, SharedBackend};
use robotics_kinematics::ArmModel;
use robotics_motion::MotionPlanner;
use robotics_simulation::SimulationBackend;
use tracing::warn;
use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    let arm = ArmModel::sg90_default();
    let mut backend = SimulationBackend::new(arm, vec![]);
    backend.start().await?;

    // Record every command issued through /ws/control to a JSONL file.
    // Operators can later replay it with `cargo run -p robotics-cli --
    // replay --from <path>`.
    let audit_path = std::env::var("ROBOTICS_AUDIT_LOG")
        .unwrap_or_else(|_| "/tmp/robotics-audit.jsonl".to_string());
    let recorder = AuditRecorder::open(&audit_path).await?;
    eprintln!("  ▸ audit log: {audit_path}");

    let dashboard = Dashboard::new(Box::new(backend), arm).with_audit(recorder);

    // Drive the arm through a loop of reachable waypoints so the
    // dashboard shows visible motion the instant a browser connects.
    tokio::spawn(demo_loop(dashboard.shared_backend(), arm));

    let addr = "0.0.0.0:8080".parse()?;
    eprintln!();
    eprintln!("  ▸ dashboard running.  open http://localhost:8080");
    eprintln!("  ▸ press Ctrl-C to stop");
    eprintln!();
    dashboard.serve(addr).await
}

/// Cycles the end-effector through a small set of waypoints. Holds the
/// backend lock only long enough to plan + apply (microseconds), then
/// sleeps so the sim and the telemetry pump can run freely.
async fn demo_loop(backend: SharedBackend, arm: ArmModel) {
    let planner = MotionPlanner::new(arm);
    // Targets in the (x, y, z) workspace — picked to stay inside the
    // sg90 default reach and within the joint limits in configs/arm.toml.
    let waypoints = [
        Vec3::new(0.12, 0.00, 0.10),
        Vec3::new(0.10, 0.06, 0.08),
        Vec3::new(0.14, 0.00, 0.06),
        Vec3::new(0.10, -0.06, 0.08),
    ];

    let mut i = 0usize;
    loop {
        let target = waypoints[i % waypoints.len()];
        i += 1;

        let plan = {
            let mut b = backend.lock().await;
            let current = match b.arm().joint_state().await {
                Ok(s) => s,
                Err(e) => {
                    warn!(?e, "demo: joint_state failed");
                    drop(b);
                    tokio::time::sleep(Duration::from_secs(1)).await;
                    continue;
                }
            };
            planner.plan_to_pose(current, target, -std::f64::consts::FRAC_PI_2, 0.5)
        };

        match plan {
            Ok(traj) => {
                let _ = backend.lock().await.arm().apply_state(traj.end).await;
            }
            Err(e) => {
                warn!(?target, ?e, "demo: plan failed; skipping waypoint");
            }
        }

        // Sleep longer than the longest plausible trajectory so the
        // sim has time to actually reach the target before the next move.
        tokio::time::sleep(Duration::from_secs(3)).await;
    }
}
