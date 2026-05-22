//! Real-time top-down ASCII visualization of the pick-and-place
//! running on the in-process simulator. Each frame redraws the
//! workspace as a 41x21 grid showing the base, the end-effector
//! position, and the cubes.
//!
//! Run:
//!   cargo run --example watch -p robotics-cli
//!
//! Press Ctrl-C to stop early.

use std::time::Duration;

use robotics_core::{Backend, Vec3};
use robotics_kinematics::{forward_kinematics, ArmModel};
use robotics_planner::{PickPlaceTask, StateMachine};
use robotics_simulation::{SimObject, SimulationBackend};

const W: usize = 41;
const H: usize = 21;
const CELL: f64 = 0.012; // meters per cell -> workspace ~±0.25 m

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let model = ArmModel::sg90_default();
    let pick = Vec3::new(0.12, 0.00, 0.05);
    let place = Vec3::new(0.05, 0.10, 0.05);

    let objects = vec![
        SimObject { id: "cube_a".into(), position: pick, grasp_radius: 0.04 },
    ];
    let mut backend = SimulationBackend::new(model, objects);
    backend.start().await?;

    let task = PickPlaceTask::top_down(pick, place);
    let mut sm = StateMachine::default();

    // Drive the task in the background while we poll state for drawing.
    let mut bk_for_task = backend.clone();
    let task_handle = tokio::spawn(async move {
        task.execute(&mut bk_for_task, &model, &mut sm).await
    });

    print!("\x1B[2J"); // clear screen once
    let start = std::time::Instant::now();
    loop {
        let joints = backend.joint_state().await;
        let ee_iso = forward_kinematics(&model, &joints);
        let ee = Vec3::new(ee_iso.translation.x, ee_iso.translation.y, ee_iso.translation.z);
        let objs = backend.objects().await;

        draw(start.elapsed(), &joints, ee, &objs);

        if task_handle.is_finished() {
            // One last frame so the end-state is visible.
            tokio::time::sleep(Duration::from_millis(50)).await;
            let joints = backend.joint_state().await;
            let ee_iso = forward_kinematics(&model, &joints);
            let ee = Vec3::new(ee_iso.translation.x, ee_iso.translation.y, ee_iso.translation.z);
            let objs = backend.objects().await;
            draw(start.elapsed(), &joints, ee, &objs);
            break;
        }
        tokio::time::sleep(Duration::from_millis(80)).await;
    }

    let _ = task_handle.await;
    backend.shutdown().await?;
    println!("\ndone.");
    Ok(())
}

fn draw(
    elapsed: Duration,
    joints: &robotics_core::JointState,
    ee: Vec3,
    objects: &[SimObject],
) {
    let mut grid = vec![vec![' '; W]; H];

    // border
    for x in 0..W {
        grid[0][x] = '─';
        grid[H - 1][x] = '─';
    }
    for y in 0..H {
        grid[y][0] = '│';
        grid[y][W - 1] = '│';
    }
    grid[0][0] = '┌';
    grid[0][W - 1] = '┐';
    grid[H - 1][0] = '└';
    grid[H - 1][W - 1] = '┘';

    // axes through origin
    let (ox, oy) = to_cell(0.0, 0.0);
    for x in 1..W - 1 {
        grid[oy][x] = '·';
    }
    for y in 1..H - 1 {
        grid[y][ox] = '·';
    }

    // cubes
    for obj in objects {
        let (cx, cy) = to_cell(obj.position.x, obj.position.y);
        if in_bounds(cx, cy) {
            grid[cy][cx] = '#';
        }
    }

    // base
    grid[oy][ox] = 'B';

    // end-effector
    let (ex, ey) = to_cell(ee.x, ee.y);
    if in_bounds(ex, ey) {
        grid[ey][ex] = '●';
    }

    // home cursor (no full clear) so frames overdraw smoothly
    print!("\x1B[H");
    println!("robotics-platform — top-down view  (B=base  ●=end-effector  #=cube)");
    println!("t={:>5.2}s  base={:+.2}  shoulder={:+.2}  elbow={:+.2}  wrist={:+.2}  grip={:+.2}      ",
        elapsed.as_secs_f64(),
        joints.base, joints.shoulder, joints.elbow, joints.wrist, joints.gripper,
    );
    println!("EE=({:+.3}, {:+.3}, {:+.3}) m                                            ",
        ee.x, ee.y, ee.z);
    println!();
    for row in &grid {
        let line: String = row.iter().collect();
        println!("{line}");
    }
}

fn to_cell(x_m: f64, y_m: f64) -> (usize, usize) {
    // map base frame: +X forward (right on screen), +Y left (up on screen)
    let cx = ((W as f64) / 2.0 + x_m / CELL).round() as isize;
    let cy = ((H as f64) / 2.0 - y_m / CELL).round() as isize;
    (cx.clamp(0, W as isize - 1) as usize, cy.clamp(0, H as isize - 1) as usize)
}

fn in_bounds(x: usize, y: usize) -> bool {
    x < W && y < H
}
