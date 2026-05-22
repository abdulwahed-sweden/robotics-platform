//! Easing functions.
//!
//! An easing function `f(t)` maps `t ∈ [0,1]` to a smoothed parameter
//! also in `[0,1]` with `f(0)=0` and `f(1)=1`. The motion planner
//! interpolates joint angles as `lerp(start, end, f(t))`, which means
//! the choice of `f` controls the *velocity profile* of the motion:
//!
//! * **Linear** — constant velocity, instantaneous accel/decel. Fine
//!   for tests, terrible for hardware (kicks the motor).
//! * **Cubic (smoothstep)** — zero velocity at endpoints, smooth
//!   accel. Acceleration is discontinuous at endpoints. Good default.
//! * **Quintic (smootherstep)** — zero velocity AND zero acceleration
//!   at endpoints. The "5th-order polynomial" you see in robotics
//!   textbooks. Use for delicate motions (carrying things).
//!
//! These are the same functions used in CG (Ken Perlin's smoothstep
//! and smootherstep). Same shape, same reason: smooth derivatives at
//! the boundaries.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Easing {
    Linear,
    /// Cubic smoothstep: 3t² - 2t³.
    Cubic,
    /// Quintic smootherstep: 6t⁵ - 15t⁴ + 10t³.
    Quintic,
}

impl Easing {
    pub fn apply(self, t: f64) -> f64 {
        let t = t.clamp(0.0, 1.0);
        match self {
            Easing::Linear => t,
            Easing::Cubic => t * t * (3.0 - 2.0 * t),
            Easing::Quintic => t * t * t * (t * (t * 6.0 - 15.0) + 10.0),
        }
    }
}

/// Boxed easing callable, for callers that want to plug in their own
/// curve (e.g. an S-curve with parameterized jerk limit) without
/// touching this enum.
pub type EasingFn = Box<dyn Fn(f64) -> f64 + Send + Sync>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn easing_endpoints_pin() {
        for e in [Easing::Linear, Easing::Cubic, Easing::Quintic] {
            assert_eq!(e.apply(0.0), 0.0);
            assert_eq!(e.apply(1.0), 1.0);
        }
    }

    #[test]
    fn cubic_midpoint_is_half() {
        // 3·0.25 - 2·0.125 = 0.75 - 0.25 = 0.5
        assert!((Easing::Cubic.apply(0.5) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn easing_is_monotonic() {
        let mut last = -1.0;
        for i in 0..=100 {
            let t = i as f64 / 100.0;
            let v = Easing::Quintic.apply(t);
            assert!(v >= last);
            last = v;
        }
    }
}
