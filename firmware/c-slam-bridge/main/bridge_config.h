#pragma once

// Parameters of the SLAM bridge. The hardware is in components/rover/include/rover_config.h.

#define ODOM_PERIOD_MS 20          // odometry packets at 50 Hz
#define CMD_TIMEOUT_MS 500         // stop when no command came for this long
#define MAX_SPEED 0.30f            // limits on the commanded velocities [m/s], [rad/s]
#define MAX_TURN_RATE 2.5f
#define LINEAR_ACCEL 0.50f         // [m/s^2]
#define ANGULAR_ACCEL 6.0f         // [rad/s^2]
#define LIDAR_BAUDRATE 115200
#define LIDAR_CHUNK 512            // the most X2 bytes in one packet
#define LIDAR_FLUSH_MS 20          // send what came at least this often
