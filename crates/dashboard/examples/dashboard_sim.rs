//! Run the dashboard against the in-process simulator.
//!
//! ```bash
//! cargo run -p robotics-dashboard --example dashboard_sim
//! # open http://localhost:8080
//! ```

use anyhow::Result;
use robotics_core::Backend;
use robotics_dashboard::Dashboard;
use robotics_kinematics::ArmModel;
use robotics_simulation::SimulationBackend;
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

    Dashboard::new(Box::new(backend), arm)
        .serve("127.0.0.1:8080".parse()?)
        .await
}
