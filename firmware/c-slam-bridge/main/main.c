// SLAM bridge: the rover as a Wi-Fi sensor and actuator node for a host running SLAM
// (host/ruby-slam).
//
// After joining Wi-Fi it listens on UDP (protocol.h). Once a host sends it a packet, it streams
// wheel odometry every ODOM_PERIOD_MS and the YDLIDAR X2's raw bytes to that host, and drives at
// the velocity the host commands. Unlike c-square it never moves on its own: with no command for
// CMD_TIMEOUT_MS it stops. The BOOT button is an emergency stop; press it again to release.
#include <math.h>

#include "bridge_config.h"
#include "button.h"
#include "drive.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"
#include "link.h"
#include "protocol.h"
#include "rover_config.h"
#include "wifi.h"

static const char *TAG = "bridge";

#define DT (CONTROL_PERIOD_MS / 1000.0f)

static float clampf(float v, float lo, float hi)
{
    return fmaxf(lo, fminf(hi, v));
}

// Move cur toward target by at most step
static float approach(float cur, float target, float step)
{
    return cur + clampf(target - cur, -step, step);
}

static void control_task(void *arg)
{
    float v = 0.0f, w = 0.0f;   // commanded velocities after the acceleration limits
    bool estop = false, was_pressed = false, was_fresh = false;
    int odom_ticks = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        drive_tick(DT);

        bool pressed = button_pressed();
        if (pressed && !was_pressed) {
            estop = !estop;
            if (estop) {
                ESP_LOGW(TAG, "Emergency stop (press BOOT again to release)");
            } else {
                ESP_LOGI(TAG, "Emergency stop released");
            }
        }
        was_pressed = pressed;

        float cmd_v, cmd_w;
        bool fresh = link_command(&cmd_v, &cmd_w);
        if (fresh != was_fresh) {
            if (fresh) {
                ESP_LOGI(TAG, "Commands arriving");
            } else {
                ESP_LOGI(TAG, "No command for %d ms: stopping", CMD_TIMEOUT_MS);
            }
            was_fresh = fresh;
        }
        bool stopped = estop || !fresh;
        if (stopped) {
            if (v != 0.0f || w != 0.0f) {
                drive_stop();
                v = w = 0.0f;
            }
        } else {
            v = approach(v, clampf(cmd_v, -MAX_SPEED, MAX_SPEED), LINEAR_ACCEL * DT);
            w = approach(w, clampf(cmd_w, -MAX_TURN_RATE, MAX_TURN_RATE), ANGULAR_ACCEL * DT);
            if (v == 0.0f && w == 0.0f) {
                drive_stop();   // brake rather than hold zero speed with the PI loop
            } else {
                drive_set(v, w);
            }
        }

        if (++odom_ticks >= ODOM_PERIOD_MS / CONTROL_PERIOD_MS) {
            odom_ticks = 0;
            pose_t p = drive_pose();
            float mv, mw;
            drive_speed(&mv, &mw);
            odom_payload_t o = {
                .x = p.x,
                .y = p.y,
                .theta = p.theta,
                .v = mv,
                .w = mw,
                .flags = (stopped ? ODOM_FLAG_STOPPED : 0) | (estop ? ODOM_FLAG_ESTOP : 0),
            };
            link_send(PKT_ODOM, (uint32_t)(esp_timer_get_time() / 1000), &o, sizeof(o));
        }
    }
}

void app_main(void)
{
    drive_init();
    drive_stop();
    button_init();
    wifi_connect();
    link_init();
    lidar_start();
    // Core 1, away from the Wi-Fi stack on core 0, for a steady control period
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 10, NULL, 1);
}
