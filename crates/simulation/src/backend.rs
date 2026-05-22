//! The [`SimulationBackend`] — the platform-facing handle to the sim.
//!
//! Owns a `Mutex<SimWorld>` and a background tick task that integrates
//! the world at a fixed rate. Implements `Backend`, `RobotArm`, and
//! `Gripper` (via separate adapter structs) so it slots into the same
//! plumbing as the hardware backend.

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use robotics_core::{
    Backend, Gripper, GripperState, JointCommand, JointState, JointTelemetry, RobotArm, Result,
};
use robotics_kinematics::ArmModel;
use tokio::sync::Mutex;
use tokio::task::JoinHandle;
use tracing::{debug, info, instrument};

use crate::world::{SimObject, SimWorld};

/// Tick rate. 200 Hz is fast enough that the kinematic integration
/// looks smooth and slow enough that we don't burn CPU. The hardware
/// backend ticks at the PWM frequency (typically 50 Hz for SG90).
const TICK_HZ: f64 = 200.0;

#[derive(Clone)]
pub struct SimulationBackend {
    world: Arc<Mutex<SimWorld>>,
    tick_task: Arc<Mutex<Option<JoinHandle<()>>>>,
}

impl SimulationBackend {
    pub fn new(model: ArmModel, objects: Vec<SimObject>) -> Self {
        Self {
            world: Arc::new(Mutex::new(SimWorld::new(model, objects))),
            tick_task: Arc::new(Mutex::new(None)),
        }
    }

    /// Snapshot of all objects currently in the world. Cheap clone.
    /// Used by the vision crate's virtual-world detector.
    pub async fn objects(&self) -> Vec<SimObject> {
        self.world.lock().await.objects.clone()
    }

    pub async fn joint_state(&self) -> JointState {
        self.world.lock().await.joints
    }
}

#[async_trait]
impl Backend for SimulationBackend {
    fn name(&self) -> &'static str {
        "simulation"
    }

    fn is_real(&self) -> bool {
        false
    }

    fn arm(&mut self) -> &mut dyn RobotArm {
        self
    }

    fn gripper(&mut self) -> &mut dyn Gripper {
        self
    }

    #[instrument(skip(self))]
    async fn start(&mut self) -> Result<()> {
        let mut slot = self.tick_task.lock().await;
        if slot.is_some() {
            return Ok(());
        }
        let world = Arc::clone(&self.world);
        let dt = 1.0 / TICK_HZ;
        let period = Duration::from_secs_f64(dt);
        let handle = tokio::spawn(async move {
            let mut ticker = tokio::time::interval(period);
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                ticker.tick().await;
                let mut w = world.lock().await;
                w.tick(dt);
            }
        });
        *slot = Some(handle);
        info!(tick_hz = TICK_HZ, "simulation backend started");
        Ok(())
    }

    async fn shutdown(&mut self) -> Result<()> {
        let mut slot = self.tick_task.lock().await;
        if let Some(handle) = slot.take() {
            handle.abort();
        }
        info!("simulation backend shut down");
        Ok(())
    }
}

#[async_trait]
impl RobotArm for SimulationBackend {
    async fn joint_state(&self) -> Result<JointState> {
        Ok(self.world.lock().await.joints)
    }

    #[instrument(skip(self), fields(joint = %cmd.joint, target = cmd.target_rad))]
    async fn apply(&mut self, cmd: JointCommand) -> Result<()> {
        let mut w = self.world.lock().await;
        // Enforce the model's limits — same as hardware will do.
        let limits = match cmd.joint {
            robotics_core::JointId::Base => w.model.base_limits,
            robotics_core::JointId::Shoulder => w.model.shoulder_limits,
            robotics_core::JointId::Elbow => w.model.elbow_limits,
            robotics_core::JointId::Wrist => w.model.wrist_limits,
            robotics_core::JointId::Gripper => w.model.gripper_limits,
        };
        limits.check(cmd.joint, cmd.target_rad)?;
        w.commanded.set(cmd.joint, cmd.target_rad);
        debug!("commanded");
        Ok(())
    }

    async fn emergency_stop(&mut self) -> Result<()> {
        let mut w = self.world.lock().await;
        w.commanded = w.joints; // freeze where we are
        Ok(())
    }
}

#[async_trait]
impl Gripper for SimulationBackend {
    async fn open(&mut self) -> Result<()> {
        let mut w = self.world.lock().await;
        w.commanded.gripper = w.model.gripper_limits.max_rad;
        w.release();
        Ok(())
    }

    async fn close(&mut self) -> Result<()> {
        let mut w = self.world.lock().await;
        w.commanded.gripper = w.model.gripper_limits.min_rad;
        w.attempt_grasp();
        Ok(())
    }

    async fn state(&self) -> Result<GripperState> {
        Ok(self.world.lock().await.gripper_state)
    }
}

/// Convenience: emit telemetry for the whole arm. Useful in tests
/// and for the CLI's `--watch` mode.
pub async fn telemetry_snapshot(backend: &SimulationBackend) -> [JointTelemetry; 5] {
    let w = backend.world.lock().await;
    let mk = |id, pos: f64, cmd: f64| JointTelemetry {
        joint: id,
        position_rad: pos,
        velocity: 0.0,
        at_target: (cmd - pos).abs() < 1e-4,
    };
    [
        mk(robotics_core::JointId::Base, w.joints.base, w.commanded.base),
        mk(robotics_core::JointId::Shoulder, w.joints.shoulder, w.commanded.shoulder),
        mk(robotics_core::JointId::Elbow, w.joints.elbow, w.commanded.elbow),
        mk(robotics_core::JointId::Wrist, w.joints.wrist, w.commanded.wrist),
        mk(robotics_core::JointId::Gripper, w.joints.gripper, w.commanded.gripper),
    ]
}
