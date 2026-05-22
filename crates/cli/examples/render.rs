//! Record a pick-and-place run and view it in the browser.
//!
//! Drives the in-process simulator through the full pick-and-place,
//! samples joint state and object positions at 30 Hz into a timeline,
//! bakes the result into an HTML file (Three.js viewer in
//! `assets/viewer.html`), writes the file to /tmp, and opens it.
//!
//! Run:
//!   cargo run --example render -p robotics-cli
//!
//! Then drag in the browser window to orbit, scroll to zoom.

use std::fs;
use std::process::Command;
use std::time::{Duration, Instant};

use robotics_core::{Backend, JointState, Vec3};
use robotics_kinematics::ArmModel;
use robotics_planner::{PickPlaceTask, StateMachine};
use robotics_simulation::{SimObject, SimulationBackend};
use serde::Serialize;

#[derive(Serialize)]
struct ObjectFrame {
    id: String,
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Serialize)]
struct Frame {
    t: f64,
    base: f64,
    shoulder: f64,
    elbow: f64,
    wrist: f64,
    gripper: f64,
    objects: Vec<ObjectFrame>,
}

#[derive(Serialize)]
struct Recording {
    base_height: f64,
    l1: f64,
    l2: f64,
    l3: f64,
    frames: Vec<Frame>,
}

const VIEWER_TEMPLATE: &str = include_str!("../../../assets/viewer.html");

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let model = ArmModel::sg90_default();
    let pick = Vec3::new(0.12, 0.00, 0.05);
    let place = Vec3::new(0.05, 0.10, 0.05);

    let objects = vec![SimObject {
        id: "cube_a".into(),
        position: pick,
        grasp_radius: 0.04,
    }];

    let mut backend = SimulationBackend::new(model, objects);
    backend.start().await?;

    // Drive the task in the background; we record from the foreground.
    let mut driver = backend.clone();
    let task_handle = tokio::spawn(async move {
        let mut sm = StateMachine::default();
        PickPlaceTask::top_down(pick, place)
            .execute(&mut driver, &model, &mut sm)
            .await
    });

    let mut frames: Vec<Frame> = Vec::new();
    let start = Instant::now();
    let mut tick = tokio::time::interval(Duration::from_millis(16)); // 60 Hz
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tick.tick().await;
        let js: JointState = backend.joint_state().await;
        let objs = backend.objects().await;
        frames.push(Frame {
            t: start.elapsed().as_secs_f64(),
            base: js.base,
            shoulder: js.shoulder,
            elbow: js.elbow,
            wrist: js.wrist,
            gripper: js.gripper,
            objects: objs
                .iter()
                .map(|o| ObjectFrame {
                    id: o.id.clone(),
                    x: o.position.x,
                    y: o.position.y,
                    z: o.position.z,
                })
                .collect(),
        });
        if task_handle.is_finished() {
            break;
        }
    }
    task_handle.await??;
    backend.shutdown().await?;

    let rec = Recording {
        base_height: model.base_height,
        l1: model.l1,
        l2: model.l2,
        l3: model.l3,
        frames,
    };

    let json = serde_json::to_string(&rec)?;
    let html = VIEWER_TEMPLATE.replace("__RECORDING__", &json);
    let path = "/tmp/robotics-render.html";
    fs::write(path, &html)?;

    println!("recorded {} frames over {:.2}s", rec.frames.len(), rec.frames.last().map(|f| f.t).unwrap_or(0.0));
    println!("wrote   {path}");

    // macOS / Linux best-effort open; fall back to printing the URL.
    #[cfg(target_os = "macos")]
    let opener = "open";
    #[cfg(target_os = "linux")]
    let opener = "xdg-open";
    #[cfg(not(any(target_os = "macos", target_os = "linux")))]
    let opener: &str = "";

    if !opener.is_empty() {
        if Command::new(opener).arg(path).status().is_err() {
            println!("open this file in your browser: file://{path}");
        }
    } else {
        println!("open this file in your browser: file://{path}");
    }
    Ok(())
}
