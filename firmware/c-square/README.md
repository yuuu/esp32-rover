# c-square

Drives a square with 50 cm sides using wheel odometry only.

- Reads both encoders with the PCNT peripheral in 4× quadrature mode (1400 counts per wheel revolution).
- Estimates the pose (x, y, θ) from differential-drive odometry.
- Controls each wheel's speed with feedforward plus a PI controller, at 100 Hz.
- Goes straight, pauses, turns 90° in place, and repeats. Speed and turn rate follow a trapezoidal
  profile. While going straight it steers toward the next corner computed from the pose.

Behavior:
- The rover starts 3 s after power-on (`AUTO_START_MS`).
- Pressing the BOOT button while driving stops it. Pressing it again starts a new run.
- It keeps looping by default (`SQUARE_LAPS = 0`).

> ⚠️ The rover starts driving right after reset, including right after flashing. USB power alone
> can spin the motors, so lift the rover before flashing.

The driving parameters are in `main/square_config.h`; the hardware is in
`../components/rover/include/rover_config.h`. Build it as in [../README.md](../README.md).
