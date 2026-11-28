// Drives a square with side SQUARE_SIDE_M using only encoder odometry.
//
// Starts AUTO_START_MS after power-on. Pressing the BOOT button while driving stops the rover;
// pressing it again restarts from there. Stops after SQUARE_LAPS laps (0 runs until stopped).
// With the start point as the origin and the initial heading as the x axis, the rover heads for
// the corners (0,0) -> (L,0) -> (L,+-L) -> (0,+-L) -> (0,0) in order.
// Each side repeats "straight -> stop -> turn 90 deg in place -> stop". While driving straight it
// steers toward the next corner computed from the pose, so turn errors and lateral drift are
// corrected on the next side.
#include <math.h>
#include <stdio.h>

#include "button.h"
#include "check.h"
#include "drive.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rover_config.h"
#include "square_config.h"

static const char *TAG = "rover";

#define DT (CONTROL_PERIOD_MS / 1000.0f)
#define DEG(rad) ((rad) * 180.0f / (float)M_PI)
#define TURN_SIGN (SQUARE_TURN_LEFT ? 1.0f : -1.0f)

typedef enum { ST_STRAIGHT, ST_TURN, ST_PAUSE_BEFORE_TURN, ST_PAUSE_BEFORE_STRAIGHT, ST_DONE } state_t;

static float clampf(float v, float lo, float hi)
{
    return fmaxf(lo, fminf(hi, v));
}

static float wrap_pi(float a)
{
    return atan2f(sinf(a), cosf(a));
}

// The i-th corner of the square (i = 0..3, 4 is the same as 0)
static void corner(int i, float *x, float *y)
{
    static const float cx[] = {0, 1, 1, 0};
    static const float cy[] = {0, 0, 1, 1};
    *x = cx[i % 4] * SQUARE_SIDE_M;
    *y = cy[i % 4] * SQUARE_SIDE_M * TURN_SIGN;
}

static void run_square(void)
{
    state_t state = ST_STRAIGHT;
    int side = 0;           // index of the current side (4 per lap; corners and heading follow from it)
    float v = 0.0f;         // commanded linear speed (kept for the acceleration limit)
    float w = 0.0f;         // commanded angular velocity for turns
    int pause_ticks = 0;
    int log_ticks = 0;
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "Start: square with %.2f m sides", SQUARE_SIDE_M);
    while (state != ST_DONE) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        drive_tick(DT);
        pose_t p = drive_pose();
        if (button_pressed()) {
            ESP_LOGI(TAG, "Stopped by BOOT button");
            break;
        }

        switch (state) {
        case ST_STRAIGHT: {
            float sx, sy, tx, ty;
            corner(side, &sx, &sy);
            corner(side + 1, &tx, &ty);
            // remaining distance along the side
            float ux = (tx - sx) / SQUARE_SIDE_M, uy = (ty - sy) / SQUARE_SIDE_M;
            float remain = (tx - p.x) * ux + (ty - p.y) * uy;
            if (remain <= 0.0f) {
                drive_stop();
                v = 0.0f;
                ESP_LOGI(TAG, "Side %d done: x=%.3f y=%.3f theta=%.1f deg", side, p.x, p.y, DEG(p.theta));
                state = ST_PAUSE_BEFORE_TURN;
                pause_ticks = PAUSE_MS / CONTROL_PERIOD_MS;
                break;
            }
            // limit acceleration and keep a speed that can stop in the remaining distance (trapezoidal profile)
            v = fminf(v + LINEAR_ACCEL * DT, CRUISE_SPEED);
            v = fminf(v, sqrtf(2.0f * LINEAR_ACCEL * remain));
            v = fmaxf(v, MIN_SPEED);
            // head for the target corner; near the end the bearing is unstable, so follow the side direction
            float heading = (remain > 0.05f) ? atan2f(ty - p.y, tx - p.x) : atan2f(uy, ux);
            float err = wrap_pi(heading - p.theta);
            drive_set(v, clampf(HEADING_KP * err, -TURN_RATE, TURN_RATE));
            break;
        }
        case ST_TURN: {
            // the target heading advances +-90 deg per side (theta is continuous, so no wrapping)
            float target = (side + 1) * TURN_SIGN * (float)M_PI / 2.0f;
            float err = target - p.theta;
            if (fabsf(err) < TURN_TOLERANCE_RAD) {
                drive_stop();
                w = 0.0f;
                ESP_LOGI(TAG, "Turn %d done: theta=%.1f deg", side, DEG(p.theta));
                side++;
                if (side % 4 == 0) {
                    ESP_LOGI(TAG, "Lap %d done: x=%.3f y=%.3f (offset from start %.1f mm)", side / 4,
                             p.x, p.y, 1000.0f * hypotf(p.x, p.y));
                }
                bool finished = SQUARE_LAPS > 0 && side >= 4 * SQUARE_LAPS;
                state = finished ? ST_DONE : ST_PAUSE_BEFORE_STRAIGHT;
                pause_ticks = PAUSE_MS / CONTROL_PERIOD_MS;
                break;
            }
            float rate = fminf(fabsf(w) + ANGULAR_ACCEL * DT, TURN_RATE);
            rate = fminf(rate, sqrtf(2.0f * ANGULAR_ACCEL * fabsf(err)));
            rate = fmaxf(rate, MIN_TURN_RATE);
            w = copysignf(rate, err);
            drive_set(0.0f, w);
            break;
        }
        case ST_PAUSE_BEFORE_TURN:
        case ST_PAUSE_BEFORE_STRAIGHT:
            if (--pause_ticks <= 0) {
                state = (state == ST_PAUSE_BEFORE_TURN) ? ST_TURN : ST_STRAIGHT;
            }
            break;
        case ST_DONE:
            break;
        }

        if (++log_ticks >= 500 / CONTROL_PERIOD_MS) {
            log_ticks = 0;
            ESP_LOGI(TAG, "x=%.3f y=%.3f theta=%.1f deg", p.x, p.y, DEG(p.theta));
        }
    }

    drive_stop();
    pose_t p = drive_pose();
    ESP_LOGI(TAG, "Stopped: x=%.3f y=%.3f theta=%.1f deg", p.x, p.y, DEG(p.theta));
    button_wait_release();   // a held button would otherwise skip the next start wait
}

void app_main(void)
{
    drive_init();
    drive_stop();
    button_init();
    if (AUTO_START_MS > 0) {
        ESP_LOGI(TAG, "Starting in %d ms", AUTO_START_MS);
        vTaskDelay(pdMS_TO_TICKS(AUTO_START_MS));
    } else {
        ESP_LOGI(TAG, "Press the BOOT button to start");
        button_wait_click();
    }
    while (true) {
        if (ROVER_CHECK_MODE) {
            rover_check_wheels();
        } else {
            odom_reset();
            run_square();
        }
        ESP_LOGI(TAG, "Press the BOOT button to start");
        button_wait_click();
    }
}
