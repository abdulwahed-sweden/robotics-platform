# Kinematics

The math behind the `robotics-kinematics` crate.

## The arm

```text
       z
       │
       │     ┌──┐ end-effector
       │   ╱─┘  │
       │  / l3  │
       │ /      ▼  wrist  (pitch)
       │( ↗ l2
       │  /     ▼  elbow  (pitch)
       │ ( ↗ l1
       │  /     ▼  shoulder (pitch)
       │( -- base_height
       └──────────────  base (yaw, about z)
```

Five revolute joints:

| Joint    | Axis           | Description                                 |
|----------|----------------|---------------------------------------------|
| base     | z              | yaw, rotates the whole arm                  |
| shoulder | y (after yaw)  | pitch, raises/lowers the upper arm          |
| elbow    | y              | pitch, bends the forearm                    |
| wrist    | y              | pitch, orients the end-effector             |
| gripper  | (prismatic)    | opens/closes; not part of the kinematic chain |

Coordinate convention: REP-103. X forward, Y left, Z up. Right-handed.

## Forward kinematics

FK is the easy direction: given joint angles, walk the chain.

Because the shoulder/elbow/wrist all pitch about parallel axes (Y
after the base yaw), the three of them define a planar problem in
the *radial-vertical* plane. Let

```text
s1 = θ_shoulder
s2 = θ_shoulder + θ_elbow
s3 = θ_shoulder + θ_elbow + θ_wrist
```

Then the planar end-effector position relative to the shoulder pivot
is

```text
r =  l1 cos(s1) + l2 cos(s2) + l3 cos(s3)
z' = l1 sin(s1) + l2 sin(s2) + l3 sin(s3)
```

The base yaw rotates that into the world:

```text
x = r cos(θ_base)
y = r sin(θ_base)
z = z' + base_height
```

Tool orientation is `Rotation_z(θ_base) · Rotation_y(-s3)`. We pack
position + orientation into an `Isometry3<f64>` and return it.

Code: `crates/kinematics/src/fk.rs`.

## Inverse kinematics

IK is harder. Given the desired position `(x, y, z)` and an
*approach pitch* `φ` (the angle the tool axis makes with the
horizontal at contact — `−π/2` for top-down, `0` for horizontal),
we solve in two stages.

### Stage 1: base yaw

The arm's planar half can only reach into the half-plane in front of
the base, so the base yaw must point at the target:

```text
θ_base = atan2(y, x)
```

Singular at x = y = 0 (target directly above the base) — we leave
the base where it is in that case.

### Stage 2: 3-link planar chain

We project the target into the (radial, vertical) plane relative to
the shoulder:

```text
r       = sqrt(x² + y²)
z_s     = z - base_height
```

The end-effector lies at `(r, z_s)`. We back off by `l3` along the
approach direction to get the *wrist* position:

```text
r_w = r - l3 cos(φ)
z_w = z_s - l3 sin(φ)
```

Now it's a 2-link IK problem from the shoulder pivot to the wrist —
the classic one solved in every robotics textbook.

Let `D = (r_w² + z_w² - l1² - l2²) / (2 l1 l2)`. Then

```text
θ_elbow    = ±acos(D)
θ_shoulder = atan2(z_w, r_w) − atan2(l2 sin(θ_elbow), l1 + l2 cos(θ_elbow))
θ_wrist    = φ − θ_shoulder − θ_elbow
```

The `±` is the elbow-up / elbow-down choice. We pick **elbow-up**
because it keeps the elbow above the table — friendlier for pick
operations.

### Reachability

Before calling `acos`, we test

```text
|l1 − l2| ≤ sqrt(r_w² + z_w²) ≤ l1 + l2
```

If that fails the target is provably unreachable in the planar
sub-problem. We return `RoboticsError::Unreachable`. (We also clamp
the `acos` argument into `[−1, 1]` to tolerate floating-point drift
on the boundary.)

### Joint limits

After computing all five joint angles, we run them through each
joint's `JointLimits::check` and fail fast if anything is out of
range. The error names the joint and the requested vs. allowed
range, so the planner can decide to try a different approach pitch
or an elbow-down branch.

Code: `crates/kinematics/src/ik.rs`.

## Why analytic, not iterative

Alternatives are:

* **Jacobian pseudo-inverse / damped least squares** — generic,
  works for any chain, but iterative. Wrong tool for a 5-DOF
  open chain we have closed-form for.
* **CCD (cyclic coordinate descent)** — fast for long chains, but
  iterative.
* **FABRIK** — geometric and elegant, also iterative.

We picked closed-form because:

1. It's **deterministic and constant-time**. A motion planner that
   re-plans per tick can't afford an unbounded convergence loop.
2. It gives **useful failure signals**. Iterative solvers say "I
   didn't converge in N iterations"; analytic says "the target is
   1.7 mm beyond your maximum reach." That's actionable.
3. The chain is **simple enough** for it. The moment we add a 6th
   DOF (a wrist roll) we lose the closed form and we'll switch to
   `IKFast` or a least-squares Jacobian. The trait surface stays
   the same.

## Tests

`crates/kinematics/src/ik.rs` ships three tests:

* `ik_then_fk_roundtrip` — the defining property. Pick a target
  inside the workspace, run IK, run FK on the result, and require
  the FK output match the original target to 10⁻⁶ m.
* `ik_rejects_unreachable_target` — out-of-reach returns
  `Unreachable`, not garbage.
* `ik_base_rotates_to_face_target` — the base-yaw stage works.

These are the right tests because they're properties, not specific
numeric outputs. If the math drifts, they'll catch it without
needing to be updated for every refactor.
