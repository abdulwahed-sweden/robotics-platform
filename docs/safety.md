# Safety

The platform is small but the failure modes are real — a 5 V servo
under load can crush a finger, knock things over, or snap its own
gears. This document is the *safety contract* the platform makes
with anyone running it.

## Layers of defense

Every command goes through three independent checks before it can
move a motor:

1. **Kinematic limits** — `JointLimits::check` inside the IK solver.
   Rejects targets that would require an angle outside the
   mathematical range. Caught at plan time, before any motion.
2. **Backend limits** — both `SimulationBackend::apply` and
   `HardwareBackend::apply` re-validate against the same limits
   before storing the commanded angle. Defends against a planner
   bug that emits an out-of-range command directly.
3. **Servo calibration limits** — the hardware `ServoCalibration`
   refuses to emit a duty cycle for an angle outside its calibrated
   range. Defends against a config mistake that widens the kinematic
   limit past the mechanical one.

You have to corrupt all three to drive a servo into its endstop.

## Emergency stop

`RobotArm::emergency_stop()` is part of the trait, not a backend
extra. That guarantees:

* You can wire one button to one line of code that works in
  simulation and on hardware.
* No higher layer can forget to expose it.
* It's idempotent and never panics.

On the hardware backend, e-stop:

1. Sets an `estop` flag inside the inner state.
2. Calls `disable()` on every PWM channel. The arm goes limp.
3. Causes every subsequent `apply` to return
   `RoboticsError::EmergencyStop` until the state machine is
   `reset()`.

On the simulation backend, e-stop:

1. Sets the commanded state equal to the current state — the arm
   freezes where it is.

The state machine's `Estop` transition is universally available from
every state. There is no "I can't stop right now" path.

## Reset

After an e-stop, `StateMachine::transition(Reset)` returns the FSM
to `Idle` — but this is *only* a software reset. On hardware, the
operator should:

1. Power-cycle the servos (they may be back-driven into an awkward
   pose during the e-stop).
2. Manually move the arm to a known home posture.
3. Run `cargo run -- calibrate center` to re-zero.
4. *Then* reset the state machine.

The platform deliberately can't enforce step 1–3 — those are physical
acts. Documentation has to do the work.

## Limit philosophy

It's tempting to make the kinematic limits exactly equal to the
mechanical limits. Don't. Always keep the kinematic limit a few
degrees inside the mechanical one. The reason: floating-point
trajectories sometimes overshoot by 10⁻³ rad due to easing-curve
acceleration; landing 0.06° past the mechanical endstop will damage
the gearbox over time.

A reasonable rule:

* Mechanical limit: physically where the servo stops.
* Hardware calibration limit: mechanical − 2°.
* Kinematic limit: hardware calibration limit − 2°.

The two-layer margin means a bad trajectory shows up as a clean
software error, not a grinding noise.

## Power

* Servos off the Pi. Always.
* Bypass each servo with 100–470 µF.
* Common ground at exactly one point. Ground loops cause the most
  baffling intermittent failures.
* Always have a physical kill switch in the servo supply line.
  Software e-stop is not enough — if the controlling process
  crashes mid-motion, the servos hold their last command.

## What we don't do (and what would be needed for higher SIL)

This platform is at home for **hobby and research use**. It is *not*
SIL-3 safety-rated. To get there you'd need:

* A safety-rated PLC monitoring the e-stop button independently of
  the Pi.
* Redundant position feedback (two encoders per joint, voted).
* A safety-rated drive that disables outputs without depending on
  software.
* Formal verification or at least IEC 61508 compliance review of the
  safety-critical code paths.

If you're building toward a real industrial deployment, treat this
platform as the **logic layer** and put a proper safety layer
underneath it.
