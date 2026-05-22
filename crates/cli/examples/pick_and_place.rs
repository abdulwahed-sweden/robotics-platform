//! End-to-end pick-and-place against the in-process simulator.
//!
//! Run with:
//!
//! ```text
//! cargo run --example pick_and_place
//! RUST_LOG=info cargo run --example pick_and_place
//! ```
//!
//! Goes through the full state-machine path: Idle → Targeting →
//! Moving → Picking → Carrying → Moving → Placing → Idle.

use robotics_core::{Backend, Vec3};
use robotics_kinematics::ArmModel;
use robotics_planner::{PickPlaceTask, StateMachine};
use robotics_simulation::{SimObject, SimulationBackend};
use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")))
        .compact()
        .init();

    let model = ArmModel::sg90_default();

    let pick = Vec3::new(0.12, 0.00, 0.05);
    let place = Vec3::new(0.10, 0.08, 0.05);

    let mut backend = SimulationBackend::new(
        model,
        vec![SimObject {
            id: "cube_a".into(),
            position: pick,
            grasp_radius: 0.04,
        }],
    );

    backend.start().await?;

    let mut sm = StateMachine::default();
    let task = PickPlaceTask::top_down(pick, place);
    task.execute(&mut backend, &model, &mut sm).await?;

    let final_state = backend.arm().joint_state().await?;
    tracing::info!(?final_state, state = sm.state.as_str(), "done");
    backend.shutdown().await?;
    Ok(())
}
