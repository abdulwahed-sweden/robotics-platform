//! The core trait contract.
//!
//! These traits are *the* abstraction that lets the same motion code
//! run against a simulated arm in Gazebo and a real arm on a Raspberry
//! Pi. Every higher layer programs against these traits; every lower
//! layer (sim, hardware) implements them.
//!
//! ## Design rules
//!
//! 1. All control surface is async. Real hardware involves I²C/SPI and
//!    GPIO syscalls; sim involves channels and timers. Both fit a
//!    `Future`-returning method better than a blocking one.
//!
//! 2. Traits are object-safe where useful (we use `async_trait` for
//!    that). Object-safety lets the CLI pick a backend at runtime
//!    and stash it in a `Box<dyn Backend>`.
//!
//! 3. Backends *own* the joint state. Higher layers ask for telemetry;
//!    they don't get to mutate state directly. This means a hardware
//!    backend can refuse a command (limit hit, e-stop pressed) without
//!    desyncing the rest of the system.

use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use crate::error::Result;
use crate::joint::{JointCommand, JointId, JointState, JointTelemetry};
use crate::pose::Vec3;
use crate::telemetry::TelemetryRx;

/// A motor controller drives a single physical or simulated motor. This
/// is the lowest-level surface; everything else composes on top.
///
/// In a real system this maps to a single PWM channel + GPIO pin. In
/// simulation it maps to one integrator inside the world.
#[async_trait]
pub trait MotorController: Send + Sync {
    async fn set_pwm_duty(&mut self, duty: f64) -> Result<()>;
    async fn shutdown(&mut self) -> Result<()>;
}

/// A joint controller drives one joint of the arm. It knows the joint's
/// limits, owns the conversion from angle → motor command, and reports
/// telemetry.
#[async_trait]
pub trait JointController: Send + Sync {
    fn id(&self) -> JointId;

    /// Issue a target angle. Returns immediately; the backend executes
    /// the move asynchronously. Use `telemetry()` to observe progress.
    async fn command(&mut self, cmd: JointCommand) -> Result<()>;

    async fn telemetry(&self) -> Result<JointTelemetry>;

    /// Cut power and freeze the joint. Used by the e-stop path.
    async fn halt(&mut self) -> Result<()>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GripperState {
    Open,
    Closed,
    /// The gripper has an object attached. The simulation tracks this
    /// explicitly; on hardware this is inferred from current draw or a
    /// limit switch and reported by the gripper driver.
    Holding,
}

#[async_trait]
pub trait Gripper: Send + Sync {
    async fn open(&mut self) -> Result<()>;
    async fn close(&mut self) -> Result<()>;
    async fn state(&self) -> Result<GripperState>;
}

/// One sensor reading. Vision sensors return objects; force/torque
/// sensors return scalars; limit switches return booleans. We keep
/// the variant set small here and grow it in the `vision` crate.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorReading {
    /// Object detected at this position (base frame, meters).
    Object { id: String, position: Vec3 },
    /// Whether the gripper is currently in contact with something.
    Contact(bool),
}

#[async_trait]
pub trait Sensor: Send + Sync {
    fn name(&self) -> &str;
    async fn read(&mut self) -> Result<Vec<SensorReading>>;
}

/// The whole arm, addressed as a single device. This is the trait the
/// motion planner programs against.
#[async_trait]
pub trait RobotArm: Send + Sync {
    /// Read the current joint state. Used as the starting point for
    /// every trajectory.
    async fn joint_state(&self) -> Result<JointState>;

    /// Apply a single command. The arm enforces joint limits and may
    /// reject commands that would violate them.
    async fn apply(&mut self, cmd: JointCommand) -> Result<()>;

    /// Apply a whole-arm command. The implementation may issue these
    /// concurrently or sequentially; the contract only guarantees all
    /// are applied before the future resolves.
    async fn apply_state(&mut self, target: JointState) -> Result<()> {
        for joint in JointId::ALL {
            self.apply(JointCommand {
                joint,
                target_rad: target.get(joint),
                max_velocity: f64::INFINITY,
                max_acceleration: f64::INFINITY,
            })
            .await?;
        }
        Ok(())
    }

    /// Engage the emergency stop. Idempotent and infallible from the
    /// caller's perspective — if e-stop itself fails, the platform is
    /// in an unrecoverable state.
    async fn emergency_stop(&mut self) -> Result<()>;
}

/// A backend is the top-level handle to a complete robotic system —
/// arm + gripper + sensors. Constructing one is how the CLI picks
/// between simulation and hardware.
///
/// We intentionally do *not* require `RobotArm` and `Gripper` to be the
/// same struct: in simulation they share an internal world handle; on
/// hardware they're separate drivers.
#[async_trait]
pub trait Backend: Send + Sync {
    /// Human-readable name for logs and CLI output.
    fn name(&self) -> &'static str;

    /// Whether this backend talks to real hardware. The planner uses
    /// this to enable extra safety checks for live runs.
    fn is_real(&self) -> bool;

    fn arm(&mut self) -> &mut dyn RobotArm;
    fn gripper(&mut self) -> &mut dyn Gripper;

    /// Start any background tasks the backend needs (sim tick loop,
    /// PWM heartbeat, etc.). Must be safe to call multiple times.
    async fn start(&mut self) -> Result<()>;

    /// Cleanly shut down. Required to ensure the hardware backend
    /// drives PWM to neutral before exit.
    async fn shutdown(&mut self) -> Result<()>;

    /// Optional native telemetry stream. Backends that own a tick loop
    /// (sim, hardware) override this to return `Some(rx)` and drive
    /// the channel from inside the tick — observers then see fresh
    /// state at the loop's native rate without polling. Default is
    /// `None`; consumers fall back to calling `arm().joint_state()`.
    fn telemetry_rx(&self) -> Option<TelemetryRx> {
        None
    }
}
