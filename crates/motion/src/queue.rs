//! FIFO queue of trajectories.
//!
//! The motion planner pushes; the backend executor pops. We could
//! use `tokio::sync::mpsc` here, but a plain `VecDeque` keeps the
//! crate runtime-agnostic — the simulation backend can drive the
//! queue on its own tick loop and the hardware backend can drive it
//! from a PWM heartbeat without either committing to tokio.

use std::collections::VecDeque;

use crate::trajectory::JointTrajectory;

#[derive(Debug, Default)]
pub struct MotionQueue {
    pending: VecDeque<JointTrajectory>,
}

impl MotionQueue {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, traj: JointTrajectory) {
        self.pending.push_back(traj);
    }

    pub fn pop(&mut self) -> Option<JointTrajectory> {
        self.pending.pop_front()
    }

    pub fn peek(&self) -> Option<&JointTrajectory> {
        self.pending.front()
    }

    pub fn len(&self) -> usize {
        self.pending.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pending.is_empty()
    }

    /// Discard all pending trajectories. Used on e-stop and on
    /// `cargo run -- cancel`.
    pub fn clear(&mut self) {
        self.pending.clear();
    }
}
