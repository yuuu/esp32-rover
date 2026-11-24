#pragma once

#include "odometry.h"

void drive_init(void);
// Target linear velocity v [m/s] and angular velocity w [rad/s] (counterclockwise positive)
void drive_set(float v, float w);
// Set the targets to 0, brake, and reset the speed controller integrators
void drive_stop(void);
// Call every control period: reads the encoders, updates the pose, and controls both wheel speeds
void drive_tick(float dt);
pose_t drive_pose(void);
