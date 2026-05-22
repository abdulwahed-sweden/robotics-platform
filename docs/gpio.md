# GPIO & PWM

This document covers what the `robotics-gpio` crate does and the
physics it's faking out.

## What a PWM signal is

A pulse-width-modulated signal is a square wave with a fixed
*period* and a variable *duty cycle*. Picture this at 50 Hz:

```text
period = 20 ms
  ┌──┐                    ┌──┐
──┘  └────────────────────┘  └─── …
 1ms       19ms          1ms

duty cycle = 1ms / 20ms = 5%
```

A hobby servo treats the duration of the high pulse — *not* the
duty fraction itself — as the position command. SG90:

| pulse width | servo position |
|-------------|----------------|
| 1.0 ms      | 0° / one end    |
| 1.5 ms      | 90° / midpoint  |
| 2.0 ms      | 180° / other end |

Other servos have different ranges (some 0.5–2.5 ms). That's why
the calibration is **per-servo**.

The duty cycle is what you write to the GPIO. Conversion:

```text
duty = pulse_ms / period_ms = pulse_ms / (1000 / frequency_hz)
```

At 50 Hz that's `pulse_ms / 20`.

This is the math implemented in
`ServoCalibration::angle_to_duty(angle_rad)`:

```rust
pulse_ms = pulse_min_ms + (angle_rad - angle_min_rad) / span * (pulse_max_ms - pulse_min_ms)
duty     = pulse_ms / (1000.0 / frequency_hz)
```

## Hardware vs software PWM

A Raspberry Pi has two hardware PWM channels on the BCM2835 chip,
exposed on GPIOs 12/18 and 13/19. These are clocked by dedicated
silicon — the pulse train is rock-solid, jitter is in nanoseconds.

For more than two servos you have two choices:

1. **Software PWM.** The Pi's kernel toggles a pin in a high-priority
   thread. `rppal::gpio::OutputPin::set_pwm` does this. Adequate at
   50 Hz with light system load; at 200 Hz or under heavy load the
   jitter gets visible (servo twitches). The platform uses
   software PWM for elbow/wrist/gripper by default.

2. **A PCA9685 PWM driver IC over I²C.** 16 channels of
   hardware-clocked PWM for ~$10. The HAT version snaps onto the
   GPIO header. This is the production path; we ship direct-GPIO as
   the baseline but you should upgrade for any arm meant to do real
   work.

## Why `rppal` is Linux-only

`rppal` talks to `/dev/gpiomem` and `/sys/class/pwm` — Linux-specific
device files. macOS and Windows can't drive Pi GPIO anyway. The
`robotics-gpio` crate cfg-gates the `rppal` dependency to
`cfg(target_os = "linux")` and provides a `StubPwm` for every other
platform. The stub logs commands rather than executing them.

That means on a Mac:

```bash
cargo build -p robotics-cli           # ✓ builds the workspace
cargo run -p robotics-cli -- hardware # ✓ runs, stubs out PWM
                                      # commands appear as tracing logs
```

You can validate command timing and limit-checking on a laptop
without a Pi attached. The real PWM kicks in only on Linux/ARM.

## Permissions

By default `/sys/class/pwm` and `/dev/gpiomem` are root-only. You
want to be in the `gpio` and `pwm` groups instead:

```bash
sudo usermod -a -G gpio,pwm $USER
# log out, log back in
```

If the binary errors with "permission denied" on
`/dev/gpiomem`, that's what you forgot.

## Device-tree overlays

Hardware PWM channels need an explicit overlay:

```
# /boot/firmware/config.txt
dtoverlay=pwm-2chan
```

Software PWM works without any overlay.

## Safety, briefly

* Don't power servos from the Pi. Run them off a separate 5 V
  supply that shares a single ground point with the Pi.
* Bypass each servo's V+ to GND with a 100–470 µF capacitor. The
  stall-current spike of a small servo can otherwise drag the rail
  down enough to reset the Pi.
* `disable()` cuts the pulse train and leaves the servo unpowered —
  it will be back-driven by gravity. Park the arm before disabling.

The full safety contract is in `docs/safety.md`.
