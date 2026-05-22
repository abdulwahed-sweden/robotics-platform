//! # robotics-protocols
//!
//! Serde-defined wire format for the platform. Today there's one
//! consumer (the CLI) and it's an in-process call, so the types are
//! mostly there to define a *contract*. Tomorrow you'll want:
//!
//! * a WebSocket dashboard reading [`TelemetryFrame`] for a live view,
//! * a tablet UI publishing [`Command`] over MQTT,
//! * a remote operator pushing [`Command`] over gRPC,
//!
//! …and not having to invent the schema for each. Defining it once
//! here and tagging it with serde lets us speak JSON, MessagePack,
//! CBOR, or postcard interchangeably.

use robotics_core::{GripperState, JointState, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Command {
    Move { target: Vec3, approach_pitch: f64 },
    Pick { target: Vec3 },
    Place { target: Vec3 },
    OpenGripper,
    CloseGripper,
    Home,
    EmergencyStop,
    Reset,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryFrame {
    /// Monotonic milliseconds since the backend started.
    pub t_ms: u64,
    pub joint_state: JointState,
    pub gripper: GripperState,
    pub state: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_roundtrips_through_json() {
        let c = Command::Move { target: Vec3::new(0.1, 0.0, 0.1), approach_pitch: -1.57 };
        let s = serde_json::to_string(&c).unwrap();
        let back: Command = serde_json::from_str(&s).unwrap();
        assert!(matches!(back, Command::Move { .. }));
    }
}
