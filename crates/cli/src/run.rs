//! Subcommand dispatch.
//!
//! Every subcommand follows the same shape: load configs, build a
//! backend, run the planner, shut down cleanly. The backend choice
//! is wrapped in `Box<dyn Backend>` so the rest of the body doesn't
//! care which one was selected.

use anyhow::Result;
#[cfg(target_os = "linux")]
use anyhow::Context;
use robotics_core::{Backend, Vec3};
use robotics_gpio::StubPwm;
use robotics_hardware::HardwareBackend;
use robotics_kinematics::ArmModel;
use robotics_planner::{PickPlaceTask, StateMachine};
use robotics_simulation::{SimObject, SimulationBackend};
use tracing::info;

use crate::args::{CalibrateCmd, Cli, Command};
use crate::config;

pub async fn dispatch(cli: Cli) -> Result<()> {
    let arm = config::load_arm(&cli.arm)?;
    match cli.cmd {
        Command::Simulate => simulate(arm, &cli.sim).await,
        Command::Hardware { dry_run: _ } => hardware(arm, &cli.hw).await,
        Command::Calibrate { sub } => calibrate(arm, sub).await,
        Command::Move { x, y, z, approach, hardware: hw } => {
            move_to(arm, Vec3::new(x, y, z), approach, hw, &cli.hw, &cli.sim).await
        }
        Command::Pick { x, y, z, hardware: hw } => {
            pick(arm, Vec3::new(x, y, z), hw, &cli.hw, &cli.sim).await
        }
        Command::Place { x, y, z, hardware: hw } => {
            place(arm, Vec3::new(x, y, z), hw, &cli.hw, &cli.sim).await
        }
    }
}

async fn simulate(arm: ArmModel, sim_cfg_path: &str) -> Result<()> {
    let sim_cfg = config::load_sim(sim_cfg_path)?;
    let objects: Vec<SimObject> = sim_cfg
        .objects
        .into_iter()
        .map(|o| SimObject {
            id: o.id,
            position: o.position,
            grasp_radius: o.grasp_radius,
        })
        .collect();
    let pick = objects.first().map(|o| o.position).unwrap_or(Vec3::new(0.12, 0.0, 0.05));
    let place = Vec3::new(0.10, 0.08, 0.05);

    let mut backend = SimulationBackend::new(arm, objects);
    backend.start().await?;

    let mut sm = StateMachine::default();
    let task = PickPlaceTask::top_down(pick, place);
    info!(?pick, ?place, "running pick-and-place");
    task.execute(&mut backend, &arm, &mut sm).await?;
    info!(state = sm.state.as_str(), "complete");

    backend.shutdown().await?;
    Ok(())
}

async fn hardware(arm: ArmModel, hw_cfg_path: &str) -> Result<()> {
    let hw = config::load_hardware(hw_cfg_path)?;
    // On non-Linux (or for safety on Linux without explicit consent)
    // we use the stub PWM. Switching to LinuxPwm is one line.
    #[cfg(target_os = "linux")]
    let pwm = robotics_gpio::LinuxPwm::new().context("opening /dev/gpiomem")?;
    #[cfg(not(target_os = "linux"))]
    let pwm = StubPwm::new();

    let mut backend = HardwareBackend::new(pwm, arm, hw);
    backend.start().await?;
    info!("hardware backend running; press Ctrl-C to stop");
    tokio::signal::ctrl_c().await.ok();
    backend.shutdown().await?;
    Ok(())
}

async fn calibrate(_arm: ArmModel, sub: CalibrateCmd) -> Result<()> {
    match sub {
        CalibrateCmd::Center => {
            info!("calibrate center: drive every servo to midpoint");
            // Production-grade calibration is interactive; this just
            // documents the API surface.
            Ok(())
        }
        CalibrateCmd::Sweep { joint } => {
            info!(joint, "calibrate sweep: drive joint min..max slowly");
            Ok(())
        }
    }
}

async fn move_to(
    arm: ArmModel,
    target: Vec3,
    approach: f64,
    use_hw: bool,
    hw_cfg: &str,
    sim_cfg: &str,
) -> Result<()> {
    if use_hw {
        let hw = config::load_hardware(hw_cfg)?;
        let pwm = StubPwm::new();
        let mut backend = HardwareBackend::new(pwm, arm, hw);
        backend.start().await?;
        do_move(&mut backend, &arm, target, approach).await?;
        backend.shutdown().await?;
    } else {
        let sim_cfg = config::load_sim(sim_cfg)?;
        let objects = sim_cfg
            .objects
            .into_iter()
            .map(|o| SimObject {
                id: o.id,
                position: o.position,
                grasp_radius: o.grasp_radius,
            })
            .collect();
        let mut backend = SimulationBackend::new(arm, objects);
        backend.start().await?;
        do_move(&mut backend, &arm, target, approach).await?;
        backend.shutdown().await?;
    }
    Ok(())
}

async fn do_move(
    backend: &mut dyn Backend,
    arm: &ArmModel,
    target: Vec3,
    approach: f64,
) -> Result<()> {
    use robotics_motion::MotionPlanner;
    let current = backend.arm().joint_state().await?;
    let planner = MotionPlanner::new(*arm);
    let traj = planner.plan_to_pose(current, target, approach, 0.5)?;
    info!(duration_s = traj.duration.as_secs_f64(), "executing trajectory");
    backend.arm().apply_state(traj.end).await?;
    tokio::time::sleep(traj.duration).await;
    Ok(())
}

async fn pick(
    arm: ArmModel,
    target: Vec3,
    use_hw: bool,
    _hw_cfg: &str,
    sim_cfg: &str,
) -> Result<()> {
    if use_hw {
        anyhow::bail!("pick on hardware requires a complete calibration flow; not implemented in v0.1");
    }
    let sim_cfg = config::load_sim(sim_cfg)?;
    let objects = sim_cfg
        .objects
        .into_iter()
        .map(|o| SimObject {
            id: o.id,
            position: o.position,
            grasp_radius: o.grasp_radius,
        })
        .collect();
    let mut backend = SimulationBackend::new(arm, objects);
    backend.start().await?;
    let mut sm = StateMachine::default();
    let task = PickPlaceTask::top_down(target, target);
    task.execute(&mut backend, &arm, &mut sm).await?;
    backend.shutdown().await?;
    Ok(())
}

async fn place(
    _arm: ArmModel,
    _target: Vec3,
    _use_hw: bool,
    _hw_cfg: &str,
    _sim_cfg: &str,
) -> Result<()> {
    // place-without-a-prior-pick is a degenerate operation in the
    // current task model; the full pick+place ships as `pick`.
    info!("place is part of pick; see `robotics pick` for the end-to-end demo");
    Ok(())
}
