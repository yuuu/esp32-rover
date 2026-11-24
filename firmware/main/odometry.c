// Dead reckoning for a differential-drive robot
#include "odometry.h"

#include <math.h>

#include "config.h"

static pose_t s_pose;

void odom_reset(void)
{
    s_pose = (pose_t){0};
}

void odom_update(float dl, float dr)
{
    float ds = (dl + dr) / 2.0f;
    float dtheta = (dr - dl) / TRACK_WIDTH_M;
    // Assume motion along the mid-interval heading (2nd-order Runge-Kutta)
    float mid = s_pose.theta + dtheta / 2.0f;
    s_pose.x += ds * cosf(mid);
    s_pose.y += ds * sinf(mid);
    s_pose.theta += dtheta;
}

pose_t odom_get(void)
{
    return s_pose;
}
