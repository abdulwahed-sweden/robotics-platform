# Hardware

How to bring the platform up on a Raspberry Pi with hobby servos.

## Bill of materials

* Raspberry Pi 4 (any RAM tier — we use <100 MB).
* 5 × SG90 (or MG90S — metal gear is much better) servos.
* External 5 V / 3 A power supply for the servos. **Do not power them
  from the Pi's 5 V rail.** They sag and the Pi browns out.
* A 100–470 µF capacitor across each servo's V+ / GND.
* PCA9685 16-channel servo HAT (optional, recommended for production —
  it gives you 16 hardware-timed PWM channels over I²C and skips the
  software-PWM jitter problem entirely).

## Wiring (without HAT)

| Joint    | Pi pin (BCM) | Notes                                  |
|----------|--------------|----------------------------------------|
| base     | GPIO 18      | Hardware PWM channel 0                 |
| shoulder | GPIO 19      | Hardware PWM channel 1                 |
| elbow    | GPIO 17      | Software PWM                           |
| wrist    | GPIO 27      | Software PWM                           |
| gripper  | GPIO 22      | Software PWM                           |

All servo grounds tie to a single point that connects to the Pi's
ground. Don't run the servo current path through the Pi.

The mapping is encoded in `configs/hardware.toml`; change pins there,
not in code.

## Wiring (with PCA9685 HAT)

The HAT plugs onto the GPIO header. Wire servos to its 16 output
headers. The current `robotics-gpio` crate doesn't ship a PCA9685
driver out of the box — adding one is a new file implementing the
`Pwm` trait and a TOML option to select it. ~200 LOC. Definitely the
production path; we ship direct-GPIO drivers as the "minimum
hardware" baseline.

## OS prep

```bash
# Enable I²C and SPI (only needed if using the HAT)
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0

# Hardware PWM via /sys/class/pwm
echo "dtoverlay=pwm-2chan" | sudo tee -a /boot/firmware/config.txt
sudo reboot

# udev rule so non-root can write /dev/gpiomem
sudo usermod -a -G gpio,pwm $USER
```

The `rppal` docs at <https://docs.rs/rppal> have authoritative setup
notes. They are short and worth reading.

## First run

```bash
# On the Pi, in this repo:
cargo build --release -p robotics-cli
./target/release/robotics calibrate center
```

That drives every servo to the midpoint of its calibrated range. The
arm should sit in a "natural" home posture. If it doesn't, your
calibration is off — proceed to the next section.

## Calibration

Two passes:

1. **Mechanical zero.** Power off, unscrew each servo horn, refit it
   so the link sits at the geometric zero of the model (forearm
   horizontal, etc.) when the servo is at its midpoint pulse. This
   makes the radian-to-pulse mapping match the kinematic model in
   `configs/arm.toml`.

2. **Endstop sweep.** Run `cargo run -- calibrate sweep --joint
   shoulder` (and so on per joint). The CLI walks the servo from min
   to max in small steps; you watch and record the angles at which
   the link hits its physical stop. Those become the `min_rad` /
   `max_rad` in `configs/arm.toml`. The hardware backend re-checks
   these every command, so a bad calibration becomes a clean error
   rather than a snapped servo gear.

The calibration helpers (`crates/hardware/src/calibration.rs`)
include a `bisect` routine that's useful if you're scripting the
sweep with a protractor — see the source for the protocol.

## Behavior of the hardware backend

* `start()` initializes PWM on every channel, then drives every servo
  to its midpoint. After that the tick task runs at 50 Hz (the SG90
  carrier frequency) and integrates current toward commanded at the
  per-joint velocity limit from `arm.toml`.
* `apply(JointCommand)` validates against the joint limits and stores
  the new commanded angle. The tick task picks it up next cycle.
* `shutdown()` disables every PWM channel. The servos go limp. Don't
  call this with the arm extended over something fragile.
* `emergency_stop()` engages an e-stop flag. Every channel is
  disabled immediately and subsequent commands return
  `RoboticsError::EmergencyStop` until you `reset` the state machine.

## Open-loop reality

SG90 has no position feedback. The "current position" the backend
reports is integrated from the commanded velocity — it's an estimate.
For real position feedback, swap in:

* **MG996R with a potentiometer wire** — the servo's internal pot is
  exposed on some clones; you read it via ADC.
* **A Dynamixel-class smart servo** (Robotis XL-330, FeeTech STS3215) —
  serial protocol, real position telemetry. Rewrite the gpio crate
  to speak that protocol.
* **AS5048 magnetic encoders** glued to each joint pivot. SPI bus.
  This is what most "open-source robot arm" projects upgrade to.

The trait surface doesn't change — only the implementation under
`HardwareBackend::joint_state` does.

## Safety

A short summary; the long form is in `docs/safety.md`.

* All commands are limit-checked twice: at the kinematics layer
  (mathematical) and at the hardware backend (mechanical
  calibration). The two limits aren't necessarily equal.
* The PWM channels are disabled on shutdown. Don't catch a SIGKILL
  somewhere that bypasses `shutdown()`.
* The e-stop is part of the `RobotArm` trait — universal across
  sim and hardware. Wire it to a physical button (a GPIO interrupt)
  and your "kill switch" is one line of code.
* The arm should always be calibrated with the servos under-powered
  before you trust it under load.
