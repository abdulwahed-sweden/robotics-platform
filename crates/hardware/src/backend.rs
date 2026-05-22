//! The real-hardware backend.
//!
//! Symmetric with `SimulationBackend`: same trait impls, different
//! actuation. Joint state is modeled by integrating the commanded
//! velocity (open-loop, since SG90 has no feedback). Replace with
//! encoder reads when you upgrade to smart servos.

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use robotics_core::{
    Backend, Gripper, GripperState, JointCommand, JointId, JointState, RobotArm, Result,
    RoboticsError,
};
use robotics_gpio::{Pwm, PwmChannel, ServoCalibration};
use robotics_kinematics::ArmModel;
use tokio::sync::Mutex;
use tokio::task::JoinHandle;
use tracing::{info, instrument, warn};

use crate::config::HardwareConfig;

/// PWM heartbeat. 50 Hz matches the SG90 frame rate; the tick task
/// updates `current` from `commanded` at the same rate as the carrier.
const TICK_HZ: f64 = 50.0;

/// Inner state, protected by a Mutex.
struct Inner<P: Pwm> {
    pwm: P,
    model: ArmModel,
    hw: HardwareConfig,
    current: JointState,
    commanded: JointState,
    estop: bool,
}

pub struct HardwareBackend<P: Pwm + 'static> {
    inner: Arc<Mutex<Inner<P>>>,
    tick: Arc<Mutex<Option<JoinHandle<()>>>>,
}

impl<P: Pwm + 'static> HardwareBackend<P> {
    pub fn new(pwm: P, model: ArmModel, hw: HardwareConfig) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Inner {
                pwm,
                model,
                hw,
                current: JointState::default(),
                commanded: JointState::default(),
                estop: false,
            })),
            tick: Arc::new(Mutex::new(None)),
        }
    }

    /// Walk every joint and command an initial mid-range angle.
    /// Called from `start()` so the arm is in a known state when
    /// telemetry starts.
    async fn home(&self) -> Result<()> {
        let mut inner = self.inner.lock().await;
        let model = inner.model;
        let mid = JointState::new(
            mid(model.base_limits.min_rad, model.base_limits.max_rad),
            mid(model.shoulder_limits.min_rad, model.shoulder_limits.max_rad),
            mid(model.elbow_limits.min_rad, model.elbow_limits.max_rad),
            mid(model.wrist_limits.min_rad, model.wrist_limits.max_rad),
            mid(model.gripper_limits.min_rad, model.gripper_limits.max_rad),
        );
        inner.commanded = mid;
        // Push initial duties so the servos snap to position.
        write_all(&mut inner)?;
        info!(?mid, "homed");
        Ok(())
    }
}

fn mid(a: f64, b: f64) -> f64 {
    (a + b) / 2.0
}

fn channel_for(hw: &HardwareConfig, joint: JointId) -> (PwmChannel, ServoCalibration) {
    let cfg = match joint {
        JointId::Base => &hw.base,
        JointId::Shoulder => &hw.shoulder,
        JointId::Elbow => &hw.elbow,
        JointId::Wrist => &hw.wrist,
        JointId::Gripper => &hw.gripper,
    };
    (cfg.channel, cfg.calibration)
}

fn write_all<P: Pwm>(inner: &mut Inner<P>) -> Result<()> {
    if inner.estop {
        return Err(RoboticsError::EmergencyStop);
    }
    for joint in JointId::ALL {
        let (channel, cal) = channel_for(&inner.hw, joint);
        let angle = inner.commanded.get(joint);
        let duty = cal.angle_to_duty(angle)?;
        inner.pwm.set_duty(channel, duty)?;
    }
    Ok(())
}

#[async_trait]
impl<P: Pwm + 'static> Backend for HardwareBackend<P> {
    fn name(&self) -> &'static str {
        "hardware"
    }
    fn is_real(&self) -> bool {
        true
    }
    fn arm(&mut self) -> &mut dyn RobotArm {
        self
    }
    fn gripper(&mut self) -> &mut dyn Gripper {
        self
    }

    #[instrument(skip(self))]
    async fn start(&mut self) -> Result<()> {
        // Initialize PWM frequency on every channel before the first
        // duty write — rppal errors out otherwise.
        {
            let mut inner = self.inner.lock().await;
            for joint in JointId::ALL {
                let (channel, cal) = channel_for(&inner.hw, joint);
                inner.pwm.set_frequency(channel, cal.frequency_hz)?;
            }
        }
        self.home().await?;

        let inner = Arc::clone(&self.inner);
        let dt = 1.0 / TICK_HZ;
        let period = Duration::from_secs_f64(dt);
        let handle = tokio::spawn(async move {
            let mut ticker = tokio::time::interval(period);
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                ticker.tick().await;
                let mut g = inner.lock().await;
                if g.estop {
                    continue;
                }
                // Integrate current toward commanded at the model's
                // per-joint velocity limits. This is our open-loop
                // "feedback".
                let model = g.model;
                let commanded = g.commanded;
                step(&mut g.current.base, commanded.base, model.base_limits.max_velocity, dt);
                step(
                    &mut g.current.shoulder,
                    commanded.shoulder,
                    model.shoulder_limits.max_velocity,
                    dt,
                );
                step(
                    &mut g.current.elbow,
                    commanded.elbow,
                    model.elbow_limits.max_velocity,
                    dt,
                );
                step(
                    &mut g.current.wrist,
                    commanded.wrist,
                    model.wrist_limits.max_velocity,
                    dt,
                );
                step(
                    &mut g.current.gripper,
                    commanded.gripper,
                    model.gripper_limits.max_velocity,
                    dt,
                );
                if let Err(e) = write_all(&mut g) {
                    warn!(?e, "pwm write failed");
                }
            }
        });
        *self.tick.lock().await = Some(handle);
        Ok(())
    }

    async fn shutdown(&mut self) -> Result<()> {
        if let Some(h) = self.tick.lock().await.take() {
            h.abort();
        }
        // Disable every channel so the arm goes limp cleanly.
        let mut inner = self.inner.lock().await;
        for joint in JointId::ALL {
            let (channel, _) = channel_for(&inner.hw, joint);
            let _ = inner.pwm.disable(channel);
        }
        info!("hardware backend shut down");
        Ok(())
    }
}

fn step(current: &mut f64, target: f64, max_vel: f64, dt: f64) {
    let delta = target - *current;
    let max_step = max_vel * dt;
    *current += delta.clamp(-max_step, max_step);
}

#[async_trait]
impl<P: Pwm + 'static> RobotArm for HardwareBackend<P> {
    async fn joint_state(&self) -> Result<JointState> {
        Ok(self.inner.lock().await.current)
    }

    async fn apply(&mut self, cmd: JointCommand) -> Result<()> {
        let mut inner = self.inner.lock().await;
        if inner.estop {
            return Err(RoboticsError::EmergencyStop);
        }
        let limits = match cmd.joint {
            JointId::Base => inner.model.base_limits,
            JointId::Shoulder => inner.model.shoulder_limits,
            JointId::Elbow => inner.model.elbow_limits,
            JointId::Wrist => inner.model.wrist_limits,
            JointId::Gripper => inner.model.gripper_limits,
        };
        limits.check(cmd.joint, cmd.target_rad)?;
        inner.commanded.set(cmd.joint, cmd.target_rad);
        Ok(())
    }

    async fn emergency_stop(&mut self) -> Result<()> {
        let mut inner = self.inner.lock().await;
        inner.estop = true;
        for joint in JointId::ALL {
            let (channel, _) = channel_for(&inner.hw, joint);
            let _ = inner.pwm.disable(channel);
        }
        warn!("emergency stop engaged");
        Ok(())
    }
}

#[async_trait]
impl<P: Pwm + 'static> Gripper for HardwareBackend<P> {
    async fn open(&mut self) -> Result<()> {
        let mut inner = self.inner.lock().await;
        inner.commanded.gripper = inner.model.gripper_limits.max_rad;
        Ok(())
    }
    async fn close(&mut self) -> Result<()> {
        let mut inner = self.inner.lock().await;
        inner.commanded.gripper = inner.model.gripper_limits.min_rad;
        Ok(())
    }
    async fn state(&self) -> Result<GripperState> {
        let inner = self.inner.lock().await;
        if (inner.current.gripper - inner.model.gripper_limits.max_rad).abs() < 0.05 {
            Ok(GripperState::Open)
        } else {
            // Open-loop: we can't actually distinguish Closed from
            // Holding. Higher layers should track this via vision.
            Ok(GripperState::Closed)
        }
    }
}
