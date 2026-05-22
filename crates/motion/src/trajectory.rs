//! Joint-space trajectories.
//!
//! A [`JointTrajectory`] is a parameterized curve from one
//! [`JointState`] to another, with a total duration that respects
//! per-joint velocity limits. The trajectory is sampleable at any
//! `t ∈ [0, duration]`.
//!
//! ## Why joint-space, not Cartesian
//!
//! Cartesian (straight-line in space) trajectories require IK at
//! every sample and can pass through singularities. Joint-space
//! trajectories interpolate angles directly — boring path through
//! the air but guaranteed kinematically valid. The motion planner
//! does the IK *once* (start + end) and then walks in joint space.

use std::time::Duration;

use robotics_core::{JointId, JointState};
use serde::{Deserialize, Serialize};

use crate::easing::Easing;

/// A single sample along the trajectory.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TrajectorySample {
    pub time: Duration,
    pub state: JointState,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct JointTrajectory {
    pub start: JointState,
    pub end: JointState,
    pub duration: Duration,
    pub easing: Easing,
}

impl JointTrajectory {
    /// Build a trajectory whose duration is set so no joint exceeds
    /// the supplied per-joint velocity ceiling.
    ///
    /// `velocity_limits` is a slice of (joint, max_rad_per_s) pairs.
    /// We compute the *peak* velocity each joint would see at the
    /// inflection point of the easing curve and scale time accordingly.
    /// For linear easing peak = avg; for cubic peak = 1.5·avg; for
    /// quintic peak = 1.875·avg. We use the most pessimistic
    /// (quintic) factor so any easing fits within limits.
    pub fn time_scaled(
        start: JointState,
        end: JointState,
        easing: Easing,
        velocity_limits: &[(JointId, f64)],
    ) -> Self {
        const PEAK_VELOCITY_FACTOR: f64 = 1.875;

        let mut min_duration_s: f64 = 0.0;
        for (joint, max_vel) in velocity_limits {
            let delta = (end.get(*joint) - start.get(*joint)).abs();
            if delta < f64::EPSILON || *max_vel <= 0.0 {
                continue;
            }
            // delta / duration is average velocity. peak = factor * avg.
            // peak <= max_vel  =>  duration >= factor * delta / max_vel.
            let need = PEAK_VELOCITY_FACTOR * delta / *max_vel;
            min_duration_s = min_duration_s.max(need);
        }

        // A floor so zero-distance moves still take some time. Without
        // this a no-op move could divide-by-zero the sampler.
        let duration = Duration::from_secs_f64(min_duration_s.max(0.01));

        Self { start, end, duration, easing }
    }

    /// Sample at a given elapsed time. Times past the duration clamp
    /// to the end (so a stale subscriber sees the final state instead
    /// of extrapolating past it).
    pub fn sample(&self, t: Duration) -> TrajectorySample {
        let total = self.duration.as_secs_f64();
        let elapsed = t.as_secs_f64().min(total);
        let u = if total > 0.0 { elapsed / total } else { 1.0 };
        let eased = self.easing.apply(u);
        TrajectorySample {
            time: t,
            state: JointState::lerp(self.start, self.end, eased),
        }
    }

    /// Discretize the trajectory into N evenly-spaced samples. Used
    /// by backends that want to push the whole plan to the executor
    /// up front instead of polling.
    pub fn discretize(&self, steps: usize) -> Vec<TrajectorySample> {
        if steps == 0 {
            return Vec::new();
        }
        let total = self.duration.as_secs_f64();
        (0..=steps)
            .map(|i| {
                let t = Duration::from_secs_f64(total * i as f64 / steps as f64);
                self.sample(t)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trajectory_starts_at_start_and_ends_at_end() {
        let start = JointState::new(0.0, 0.0, 0.0, 0.0, 0.0);
        let end = JointState::new(1.0, 0.5, -0.3, 0.2, 0.1);
        let traj = JointTrajectory::time_scaled(
            start,
            end,
            Easing::Cubic,
            &[
                (JointId::Base, 1.0),
                (JointId::Shoulder, 1.0),
                (JointId::Elbow, 1.0),
                (JointId::Wrist, 1.0),
                (JointId::Gripper, 1.0),
            ],
        );
        let s0 = traj.sample(Duration::ZERO);
        let s1 = traj.sample(traj.duration);
        assert!((s0.state.base - start.base).abs() < 1e-9);
        assert!((s1.state.base - end.base).abs() < 1e-9);
    }

    #[test]
    fn velocity_ceiling_is_respected() {
        let start = JointState::new(0.0, 0.0, 0.0, 0.0, 0.0);
        let end = JointState::new(1.0, 0.0, 0.0, 0.0, 0.0);
        let traj = JointTrajectory::time_scaled(
            start,
            end,
            Easing::Cubic,
            &[(JointId::Base, 1.0)],
        );
        // Sample numerical velocity. Should never exceed 1.0 rad/s
        // (with a small numerical slop).
        let samples = traj.discretize(200);
        let mut peak = 0.0f64;
        for w in samples.windows(2) {
            let dt = (w[1].time.as_secs_f64() - w[0].time.as_secs_f64()).max(1e-9);
            let dv = (w[1].state.base - w[0].state.base).abs() / dt;
            peak = peak.max(dv);
        }
        assert!(peak <= 1.0 + 1e-3, "peak velocity {} exceeds 1.0", peak);
    }
}
