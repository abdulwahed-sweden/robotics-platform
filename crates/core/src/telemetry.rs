//! Lock-free single-writer / many-reader telemetry channel.
//!
//! Backends with their own tick loop (sim, hardware) publish the
//! freshest [`JointState`] here. Observers — the dashboard, a
//! recorder, a ROS bridge — subscribe and read at their own rate.
//! Latest-wins semantics: a slow reader sees the newest frame, not
//! a queue of stale ones, so the publish call from a 200 Hz tick is
//! always cheap and bounded.

use std::sync::Arc;

use tokio::sync::watch;

use crate::JointState;

pub type TelemetryRx = watch::Receiver<JointState>;

pub struct TelemetryHub {
    tx: watch::Sender<JointState>,
}

impl TelemetryHub {
    pub fn new(initial: JointState) -> (Arc<Self>, TelemetryRx) {
        let (tx, rx) = watch::channel(initial);
        (Arc::new(Self { tx }), rx)
    }

    /// Non-blocking publish. Drops nothing — it just replaces the
    /// stored value. Safe from a 200 Hz tick loop; safe with zero
    /// receivers (returns nothing useful, but won't error).
    pub fn publish(&self, state: JointState) {
        let _ = self.tx.send_replace(state);
    }

    pub fn subscribe(&self) -> TelemetryRx {
        self.tx.subscribe()
    }
}
