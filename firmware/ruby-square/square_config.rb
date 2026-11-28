# Driving parameters of the square run, the same as
# firmware/c-square/main/square_config.h.

# When true, the rover does not drive; it turns each wheel briefly and prints the encoder counts
CHECK_MODE = false

SQUARE_SIDE_M = 0.50
SQUARE_TURN_LEFT = true   # true: turn left (counterclockwise), false: turn right
SQUARE_LAPS = 0           # Number of laps. 0 runs until stopped (BOOT button stops it)
AUTO_START_MS = 3000      # Delay from power-on to start. 0 waits for the BOOT button
CRUISE_SPEED = 0.25       # Straight-line speed [m/s] (square verified at 0.15)
LINEAR_ACCEL = 0.50       # [m/s^2]
MIN_SPEED = 0.03          # Keep at least this speed right before the target
HEADING_KP = 4.0          # heading error [rad] -> angular velocity [rad/s]
TURN_RATE = 2.5           # Angular velocity for turning in place [rad/s]
ANGULAR_ACCEL = 6.0       # [rad/s^2]
MIN_TURN_RATE = 0.3
TURN_TOLERANCE_RAD = 0.01 # about 0.6 deg
PAUSE_MS = 300            # Pause between straight segments and turns
