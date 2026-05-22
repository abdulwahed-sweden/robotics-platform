//! WebSocket routes.
//!
//! * `GET /ws/telemetry` — server pushes `TelemetryFrame` JSON,
//!   one per change (coalesced to ~60 fps by the bridge).
//! * `GET /ws/control`   — client sends `Command` JSON. We dispatch
//!   to the backend and reply with a tiny `{ok, msg}` ack.
//!
//! The e-stop path is hand-coded to short-circuit before anything
//! else can interleave. Do not refactor it into the generic dispatch.

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::State;
use axum::response::IntoResponse;
use futures_util::{SinkExt, StreamExt};
use robotics_core::JointState;
use robotics_motion::MotionPlanner;
use robotics_protocols::Command;
use serde::Serialize;
use tracing::{info, warn};

use crate::state::Shared;

pub async fn telemetry(ws: WebSocketUpgrade, State(shared): State<Shared>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| telemetry_loop(socket, shared))
}

async fn telemetry_loop(socket: WebSocket, shared: Shared) {
    let (mut tx, mut rx) = socket.split();
    let mut sub = shared.tx.subscribe();
    loop {
        tokio::select! {
            frame = sub.recv() => {
                let Ok(frame) = frame else { break };
                let payload = match serde_json::to_string(&frame) {
                    Ok(p) => p,
                    Err(_) => continue,
                };
                if tx.send(Message::Text(payload)).await.is_err() {
                    break;
                }
            }
            incoming = rx.next() => {
                match incoming {
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Err(_)) => break,
                    _ => continue,
                }
            }
        }
    }
}

pub async fn control(ws: WebSocketUpgrade, State(shared): State<Shared>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| control_loop(socket, shared))
}

async fn control_loop(mut socket: WebSocket, shared: Shared) {
    while let Some(Ok(msg)) = socket.next().await {
        let raw = match msg {
            Message::Text(t) => t,
            Message::Close(_) => break,
            _ => continue,
        };
        let cmd: Command = match serde_json::from_str(&raw) {
            Ok(c) => c,
            Err(e) => {
                warn!(error = %e, "bad command JSON");
                let _ = send_ack(&mut socket, Ack::err(format!("bad json: {e}"))).await;
                continue;
            }
        };

        // E-stop short-circuit. No planner, no other locks contended.
        if matches!(cmd, Command::EmergencyStop) {
            let outcome = {
                let mut b = shared.backend.lock().await;
                b.arm().emergency_stop().await
            };
            let ack = match outcome {
                Ok(_) => Ack::ok("estop"),
                Err(e) => Ack::err(format!("estop: {e}")),
            };
            let _ = send_ack(&mut socket, ack).await;
            info!("e-stop dispatched from dashboard");
            continue;
        }

        let outcome = dispatch(&shared, &cmd).await;
        let ack = match outcome {
            Ok(()) => Ack::ok("accepted"),
            Err(e) => Ack::err(e),
        };
        let _ = send_ack(&mut socket, ack).await;
    }
}

async fn dispatch(shared: &Shared, cmd: &Command) -> Result<(), String> {
    let mut b = shared.backend.lock().await;
    match cmd {
        Command::OpenGripper => b.gripper().open().await.map_err(|e| e.to_string()),
        Command::CloseGripper => b.gripper().close().await.map_err(|e| e.to_string()),
        Command::Home => b
            .arm()
            .apply_state(JointState::default())
            .await
            .map_err(|e| e.to_string()),
        // FSM reset is observed by the state machine, not the backend.
        Command::Reset => Ok(()),
        Command::Move { target, approach_pitch } => {
            plan_and_apply(&mut b, &shared.arm, *target, *approach_pitch).await
        }
        Command::Pick { target } | Command::Place { target } => {
            plan_and_apply(&mut b, &shared.arm, *target, -std::f64::consts::FRAC_PI_2).await
        }
        Command::EmergencyStop => unreachable!("handled above"),
    }
}

async fn plan_and_apply(
    b: &mut tokio::sync::MutexGuard<'_, Box<dyn robotics_core::Backend>>,
    arm: &robotics_kinematics::ArmModel,
    target: robotics_core::Vec3,
    approach: f64,
) -> Result<(), String> {
    let current = b.arm().joint_state().await.map_err(|e| e.to_string())?;
    let planner = MotionPlanner::new(*arm);
    let traj = planner
        .plan_to_pose(current, target, approach, 0.5)
        .map_err(|e| e.to_string())?;
    b.arm().apply_state(traj.end).await.map_err(|e| e.to_string())
}

#[derive(Serialize)]
struct Ack {
    ok: bool,
    msg: String,
}

impl Ack {
    fn ok<S: Into<String>>(msg: S) -> Self {
        Self { ok: true, msg: msg.into() }
    }
    fn err<S: Into<String>>(msg: S) -> Self {
        Self { ok: false, msg: msg.into() }
    }
}

async fn send_ack(socket: &mut WebSocket, ack: Ack) -> Result<(), axum::Error> {
    let payload = serde_json::to_string(&ack).unwrap_or_else(|_| "{}".into());
    socket.send(Message::Text(payload)).await
}
