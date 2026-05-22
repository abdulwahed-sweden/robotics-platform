//! Shared state injected into every axum handler.
//!
//! The CLI's control loop and the dashboard's WS handlers both poke
//! the same `Backend`. We put it behind an async `Mutex`; the mutex
//! is held briefly per command. Telemetry consumers never block on
//! it because the bridge fans out `TelemetryFrame`s through a
//! broadcast channel, and (when the backend supports it) reads
//! state from a lock-free `watch` channel published by the tick
//! loop itself.

use std::sync::Arc;

use tokio::sync::{broadcast, Mutex};

use robotics_core::Backend;
use robotics_kinematics::ArmModel;
use robotics_protocols::TelemetryFrame;

pub type SharedBackend = Arc<Mutex<Box<dyn Backend>>>;

#[derive(Clone)]
pub struct Shared {
    pub backend: SharedBackend,
    pub arm: ArmModel,
    /// One writer (the bridge), many readers (WS subscribers). 64 slots
    /// is "drop late frames over block" — the UI is read-only.
    pub tx: broadcast::Sender<TelemetryFrame>,
}

impl Shared {
    pub fn new(backend: Box<dyn Backend>, arm: ArmModel) -> Self {
        let (tx, _) = broadcast::channel(64);
        Self {
            backend: Arc::new(Mutex::new(backend)),
            arm,
            tx,
        }
    }
}
