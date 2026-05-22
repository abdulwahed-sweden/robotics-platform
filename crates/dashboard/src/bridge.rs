//! Telemetry pump — background task that fills the broadcast channel
//! every WebSocket subscriber drains from.
//!
//! Two paths:
//!
//! * **Native** — backend exposes a `TelemetryRx` (sim, hardware). We
//!   subscribe and emit on every change, coalesced to ~60 fps so a
//!   200 Hz tick doesn't drown a browser. No lock acquired per frame.
//!
//! * **Polling fallback** — for backends without a hub. We re-acquire
//!   the backend mutex at 20 Hz and read joint state through the
//!   `RobotArm` trait. Slower, lock-heavier, but works for anything
//!   that implements `Backend`.

use std::time::{Duration, Instant};

use robotics_core::{GripperState, JointState, TelemetryRx};
use robotics_protocols::TelemetryFrame;
use tokio::time::{interval, MissedTickBehavior};

use crate::state::Shared;

pub fn spawn(shared: Shared) {
    tokio::spawn(async move {
        let rx = {
            let b = shared.backend.lock().await;
            b.telemetry_rx()
        };
        match rx {
            Some(rx) => native_pump(shared, rx).await,
            None => poll_pump(shared).await,
        }
    });
}

async fn native_pump(shared: Shared, mut rx: TelemetryRx) {
    let start = Instant::now();
    let mut tick = interval(Duration::from_millis(16)); // ~60 fps
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
        tick.tick().await;
        // Block until *something* has changed since the last emission.
        // `changed()` returning Err means the publisher dropped — which
        // for a running backend means we're shutting down.
        if rx.changed().await.is_err() {
            break;
        }
        let js = *rx.borrow();
        let frame = build_frame(&shared, js, &start).await;
        let _ = shared.tx.send(frame);
    }
}

async fn poll_pump(shared: Shared) {
    let mut tick = interval(Duration::from_millis(50));
    tick.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let start = Instant::now();
    loop {
        tick.tick().await;
        let js = {
            let mut b = shared.backend.lock().await;
            match b.arm().joint_state().await {
                Ok(s) => s,
                Err(_) => continue,
            }
        };
        let frame = build_frame(&shared, js, &start).await;
        let _ = shared.tx.send(frame);
    }
}

async fn build_frame(shared: &Shared, joint_state: JointState, start: &Instant) -> TelemetryFrame {
    let gripper = {
        let mut b = shared.backend.lock().await;
        b.gripper().state().await.unwrap_or(GripperState::Open)
    };
    TelemetryFrame {
        t_ms: start.elapsed().as_millis() as u64,
        joint_state,
        gripper,
        state: "running".into(),
    }
}
