//! # robotics-dashboard
//!
//! A small, embedded web UI for any `Backend`. Three routes:
//!
//! * `GET /`              — single-file HTML viewer (canvas + sidebar).
//! * `GET /ws/telemetry`  — server-pushed `TelemetryFrame` JSON.
//! * `GET /ws/control`    — client-sent `Command` JSON, acked.
//!
//! The dashboard never owns the backend; it shares one with whatever
//! else is driving it (the CLI, a test harness, an MQTT bridge). On
//! shutdown, just drop the future — the background pump cancels with
//! it. No audit trail or auth in this layer yet; that lands once the
//! Postgres / `rustio-admin` integration is wired in.

mod bridge;
mod state;
mod ws;

use std::net::SocketAddr;

use anyhow::Result;
use axum::http::header;
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;
use robotics_core::Backend;
use robotics_kinematics::ArmModel;
use tracing::info;

pub use state::Shared;

const INDEX_HTML: &str = include_str!("../static/index.html");

pub struct Dashboard {
    shared: Shared,
}

impl Dashboard {
    /// Build a dashboard over `backend`. The `arm` model is needed so
    /// Cartesian commands (`Move`/`Pick`/`Place`) can plan trajectories
    /// without round-tripping to the CLI.
    pub fn new(backend: Box<dyn Backend>, arm: ArmModel) -> Self {
        Self { shared: Shared::new(backend, arm) }
    }

    /// Serve until the future is dropped or the listener errors.
    pub async fn serve(self, addr: SocketAddr) -> Result<()> {
        bridge::spawn(self.shared.clone());

        let app = Router::new()
            .route("/", get(index))
            .route("/ws/telemetry", get(ws::telemetry))
            .route("/ws/control", get(ws::control))
            .with_state(self.shared);

        let listener = tokio::net::TcpListener::bind(addr).await?;
        info!(%addr, "dashboard listening");
        axum::serve(listener, app).await?;
        Ok(())
    }
}

async fn index() -> impl IntoResponse {
    ([(header::CONTENT_TYPE, "text/html; charset=utf-8")], INDEX_HTML)
}
