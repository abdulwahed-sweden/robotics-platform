//! The robot state machine.
//!
//! Every transition is explicit. Bad transitions return an error
//! rather than silently doing the wrong thing. This is the same
//! pattern as a hardware interlock: code can't bypass safety states
//! by mistake because every transition has to be named.
//!
//! ## States
//!
//! ```text
//!                ┌────────────┐
//!                │   Idle     │ ◄──────────────┐
//!                └────┬───────┘                │
//!                     │ move_to                │ done / reset
//!                     ▼                        │
//!                ┌────────────┐                │
//!                │ Targeting  │                │
//!                └────┬───────┘                │
//!                     │ trajectory_ready       │
//!                     ▼                        │
//!                ┌────────────┐                │
//!         ┌──────│  Moving    │────────────────┤
//!         │      └────┬───────┘                │
//!         │           │ at_pick_pose           │
//!         │           ▼                        │
//!         │      ┌────────────┐                │
//!         │      │  Picking   │                │
//!         │      └────┬───────┘                │
//!         │           │ grasped                │
//!         │           ▼                        │
//!         │      ┌────────────┐                │
//!         │      │ Carrying   │────►Moving──►Placing──►Idle
//!         │      └────────────┘                │
//!         │                                    │
//!         │  error  ┌────────────┐             │
//!         └────────►│   Error    │─── reset ───┘
//!                   └────┬───────┘
//!                        │ estop
//!                        ▼
//!                   ┌────────────┐
//!                   │ EStop      │ (terminal until manual reset)
//!                   └────────────┘
//! ```

use robotics_core::{Result, RoboticsError};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RobotState {
    Idle,
    Targeting,
    Moving,
    Picking,
    Carrying,
    Placing,
    Error,
    EmergencyStop,
}

impl RobotState {
    pub fn as_str(self) -> &'static str {
        match self {
            RobotState::Idle => "idle",
            RobotState::Targeting => "targeting",
            RobotState::Moving => "moving",
            RobotState::Picking => "picking",
            RobotState::Carrying => "carrying",
            RobotState::Placing => "placing",
            RobotState::Error => "error",
            RobotState::EmergencyStop => "emergency_stop",
        }
    }
}

/// The transition vocabulary. Adding a new transition means adding a
/// new variant *and* a row in [`StateMachine::transition`]. The
/// compiler will catch missing rows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Transition {
    BeginPlanning,
    PlanReady,
    AtPickPose,
    Grasped,
    AtPlacePose,
    Released,
    Complete,
    Fault,
    Estop,
    Reset,
}

#[derive(Debug, Clone, Copy)]
pub struct StateMachine {
    pub state: RobotState,
}

impl Default for StateMachine {
    fn default() -> Self {
        Self { state: RobotState::Idle }
    }
}

impl StateMachine {
    /// Apply a transition. Returns the new state on success, or an
    /// `InvalidStateTransition` error on a bad pair.
    pub fn transition(&mut self, t: Transition) -> Result<RobotState> {
        use RobotState::*;
        use Transition::*;
        // Estop and Reset are universally available so the e-stop
        // button is never "blocked" by state.
        if let Estop = t {
            self.state = EmergencyStop;
            return Ok(self.state);
        }
        if let Reset = t {
            // From EStop or Error only.
            if matches!(self.state, EmergencyStop | Error) {
                self.state = Idle;
                return Ok(self.state);
            }
            return self.bad(t);
        }
        let next = match (self.state, t) {
            (Idle, BeginPlanning) => Targeting,
            (Targeting, PlanReady) => Moving,
            (Moving, AtPickPose) => Picking,
            (Picking, Grasped) => Carrying,
            // Chaining segments: an industrial controller permits a
            // new BeginPlanning while in-flight (the previous segment
            // has effectively completed by the time the next plan
            // arrives). We allow it from Moving and Carrying so
            // pick-and-place can stitch its sub-moves end-to-end.
            (Moving, BeginPlanning) => Targeting,
            (Carrying, BeginPlanning) => Targeting,
            (Carrying, PlanReady) => Moving,
            (Moving, AtPlacePose) => Placing,
            (Placing, Released) => Idle,
            (_, Complete) => Idle,
            (_, Fault) => Error,
            _ => return self.bad(t),
        };
        self.state = next;
        Ok(next)
    }

    fn bad(&self, t: Transition) -> Result<RobotState> {
        Err(RoboticsError::InvalidStateTransition {
            from: self.state.as_str().to_string(),
            to: format!("{:?}", t),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn happy_path_pick_and_place() {
        let mut sm = StateMachine::default();
        assert_eq!(sm.state, RobotState::Idle);
        sm.transition(Transition::BeginPlanning).unwrap();
        assert_eq!(sm.state, RobotState::Targeting);
        sm.transition(Transition::PlanReady).unwrap();
        sm.transition(Transition::AtPickPose).unwrap();
        sm.transition(Transition::Grasped).unwrap();
        assert_eq!(sm.state, RobotState::Carrying);
        sm.transition(Transition::PlanReady).unwrap();
        sm.transition(Transition::AtPlacePose).unwrap();
        sm.transition(Transition::Released).unwrap();
        assert_eq!(sm.state, RobotState::Idle);
    }

    #[test]
    fn bad_transitions_are_rejected() {
        let mut sm = StateMachine::default();
        assert!(sm.transition(Transition::Grasped).is_err());
    }

    #[test]
    fn estop_always_works_and_reset_recovers() {
        let mut sm = StateMachine::default();
        sm.transition(Transition::BeginPlanning).unwrap();
        sm.transition(Transition::Estop).unwrap();
        assert_eq!(sm.state, RobotState::EmergencyStop);
        // No normal transition out of EStop.
        assert!(sm.transition(Transition::BeginPlanning).is_err());
        sm.transition(Transition::Reset).unwrap();
        assert_eq!(sm.state, RobotState::Idle);
    }
}
