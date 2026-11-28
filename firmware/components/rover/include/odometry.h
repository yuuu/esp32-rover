#pragma once

// Robot pose. The origin is the start point, x is the initial heading (forward), y is left.
// theta [rad] is counterclockwise positive and does not wrap at +-pi (continuous)
typedef struct {
    float x;
    float y;
    float theta;
} pose_t;

void odom_reset(void);
// Update the pose from the distance [m] each wheel traveled since the last call
void odom_update(float dl, float dr);
pose_t odom_get(void);
