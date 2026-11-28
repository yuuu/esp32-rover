// Wheel speed control (feedforward + PI) and pose update
#include "drive.h"

#include <math.h>
#include <stdbool.h>

#include "rover_config.h"
#include "encoder.h"
#include "motor.h"

#define M_PER_COUNT ((float)M_PI * WHEEL_DIAMETER_M / ENC_COUNTS_PER_REV)
#define INTEGRAL_LIMIT 0.1f   // Keeps the integral term within about +-0.6 duty

typedef struct {
    int32_t last_count;
    float speed;      // measured speed [m/s] (after low-pass)
    float target;     // target speed [m/s]
    float integral;
} wheel_state_t;

static wheel_state_t s_wheels[WHEEL_COUNT];
static bool s_stopped = true;

void drive_init(void)
{
    motor_init();
    encoder_init();
    odom_reset();
    for (int i = 0; i < WHEEL_COUNT; i++) {
        s_wheels[i] = (wheel_state_t){.last_count = encoder_get(i)};
    }
}

void drive_set(float v, float w)
{
    s_wheels[WHEEL_L].target = v - w * TRACK_WIDTH_M / 2.0f;
    s_wheels[WHEEL_R].target = v + w * TRACK_WIDTH_M / 2.0f;
    s_stopped = false;
}

void drive_stop(void)
{
    for (int i = 0; i < WHEEL_COUNT; i++) {
        s_wheels[i].target = 0.0f;
        s_wheels[i].integral = 0.0f;
        motor_set(i, 0.0f);
    }
    s_stopped = true;
}

void drive_tick(float dt)
{
    float dist[WHEEL_COUNT];
    for (int i = 0; i < WHEEL_COUNT; i++) {
        wheel_state_t *s = &s_wheels[i];
        int32_t count = encoder_get(i);
        dist[i] = (count - s->last_count) * M_PER_COUNT;
        s->last_count = count;
        float raw = dist[i] / dt;
        s->speed += WHEEL_SPEED_LPF * (raw - s->speed);
    }
    odom_update(dist[WHEEL_L], dist[WHEEL_R]);

    if (s_stopped) {
        return;   // stay braked
    }
    for (int i = 0; i < WHEEL_COUNT; i++) {
        wheel_state_t *s = &s_wheels[i];
        float err = s->target - s->speed;
        s->integral = fmaxf(-INTEGRAL_LIMIT, fminf(INTEGRAL_LIMIT, s->integral + err * dt));
        float duty = s->target / WHEEL_SPEED_AT_FULL_DUTY + WHEEL_KP * err + WHEEL_KI * s->integral;
        motor_set(i, duty);
    }
}

pose_t drive_pose(void)
{
    return odom_get();
}

void drive_speed(float *v, float *w)
{
    *v = (s_wheels[WHEEL_L].speed + s_wheels[WHEEL_R].speed) / 2.0f;
    *w = (s_wheels[WHEEL_R].speed - s_wheels[WHEEL_L].speed) / TRACK_WIDTH_M;
}
