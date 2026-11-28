#pragma once

// Driving parameters of the square run. The hardware (pins, dimensions, wheel control) is in
// components/rover/include/rover_config.h.

// When 1, the rover does not drive; it turns each wheel briefly and prints the encoder counts
#ifndef ROVER_CHECK_MODE
#define ROVER_CHECK_MODE 0
#endif

// ================================================================ Driving (square)
#define SQUARE_SIDE_M 0.50f
#define SQUARE_TURN_LEFT 1        // 1: turn left (counterclockwise), 0: turn right
#define SQUARE_LAPS 0             // Number of laps. 0 runs until stopped (BOOT button stops it)
#define AUTO_START_MS 3000        // Delay from power-on to start. 0 waits for the BOOT button
#define CRUISE_SPEED 0.25f        // Straight-line speed [m/s] (square verified at 0.15)
#define LINEAR_ACCEL 0.50f        // [m/s^2]
#define MIN_SPEED 0.03f           // Keep at least this speed right before the target
#define HEADING_KP 4.0f           // heading error [rad] -> angular velocity [rad/s]
#define TURN_RATE 2.5f            // Angular velocity for turning in place [rad/s]
#define ANGULAR_ACCEL 6.0f        // [rad/s^2]
#define MIN_TURN_RATE 0.3f
#define TURN_TOLERANCE_RAD 0.01f  // about 0.6 deg
#define PAUSE_MS 300              // Pause between straight segments and turns
