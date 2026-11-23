// エンコーダの自己位置推定 (オドメトリ) だけで、一辺 SQUARE_SIDE_M の正方形を回る。
//
// 電源を入れて AUTO_START_MS 後に走り出す。走行中に BOOT ボタンを押すと止まり、
// もう一度押すとそこから走り直す。SQUARE_LAPS 周で止まる (0 なら止めるまで回り続ける)。スタート地点を原点、最初の向きを x 軸として、
// 正方形の角 (0,0) → (L,0) → (L,±L) → (0,±L) → (0,0) を順に目指す。
// 各辺は「直進 → 停止 → その場で90°旋回 → 停止」の繰り返し。直進中は自己位置から
// 次の角への向きを計算して舵を切るので、旋回の誤差や横ずれも次の辺で戻そうとする。
#include <math.h>
#include <stdio.h>

#include "config.h"
#include "drive.h"
#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"

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

// 正方形の i 番目の角 (i = 0..3、4 は 0 と同じ)
static void corner(int i, float *x, float *y)
{
    static const float cx[] = {0, 1, 1, 0};
    static const float cy[] = {0, 0, 1, 1};
    *x = cx[i % 4] * SQUARE_SIDE_M;
    *y = cy[i % 4] * SQUARE_SIDE_M * TURN_SIGN;
}

static void button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_START_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&cfg);
}

static void wait_start_button(void)
{
    ESP_LOGI(TAG, "BOOT ボタンを押すとスタートします");
    while (gpio_get_level(PIN_START_BUTTON) != 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    while (gpio_get_level(PIN_START_BUTTON) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));   // 手を離してから走り出す
}

static bool button_pressed(void)
{
    return gpio_get_level(PIN_START_BUTTON) == 0;
}

// 左右の車輪を順に少し回し、カウントの増え方を表示する (config.h の向きの確認用)
static void check_wheels(void)
{
    static const char *names[] = {"左", "右"};
    for (int w = 0; w < WHEEL_COUNT; w++) {
        int32_t before = encoder_get(w);
        ESP_LOGI(TAG, "%s の車輪を前へ回します (1秒)", names[w]);
        motor_set(w, 0.3f);
        vTaskDelay(pdMS_TO_TICKS(1000));
        motor_set(w, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(300));
        int32_t delta = encoder_get(w) - before;
        ESP_LOGI(TAG, "%s: カウント %+ld (%.2f 回転)", names[w], (long)delta,
                 (float)delta / ENC_COUNTS_PER_REV);
    }
    ESP_LOGI(TAG, "確認: 回ったのは名前どおりの車輪か (違えば LEFT_IS_A)、前へ回ったか "
                  "(違えば MOTOR_*_INVERT)、カウントは正か (違えば ENC_*_INVERT)");
}

static void run_square(void)
{
    state_t state = ST_STRAIGHT;
    int side = 0;           // 今走っている辺の通し番号 (4 で 1 周。角と向きは side から決まる)
    float v = 0.0f;         // 直進の指令速度 (加速の制限用に前回値を持つ)
    float w = 0.0f;         // 旋回の指令角速度
    int pause_ticks = 0;
    int log_ticks = 0;
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "スタート: 一辺 %.2f m の正方形", SQUARE_SIDE_M);
    while (state != ST_DONE) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        drive_tick(DT);
        pose_t p = drive_pose();
        if (button_pressed()) {
            ESP_LOGI(TAG, "BOOT ボタンで停止");
            break;
        }

        switch (state) {
        case ST_STRAIGHT: {
            float sx, sy, tx, ty;
            corner(side, &sx, &sy);
            corner(side + 1, &tx, &ty);
            // 辺の向きに沿った残り距離
            float ux = (tx - sx) / SQUARE_SIDE_M, uy = (ty - sy) / SQUARE_SIDE_M;
            float remain = (tx - p.x) * ux + (ty - p.y) * uy;
            if (remain <= 0.0f) {
                drive_stop();
                v = 0.0f;
                ESP_LOGI(TAG, "辺 %d 終了: x=%.3f y=%.3f θ=%.1f°", side, p.x, p.y, DEG(p.theta));
                state = ST_PAUSE_BEFORE_TURN;
                pause_ticks = PAUSE_MS / CONTROL_PERIOD_MS;
                break;
            }
            // 加速の制限と、残り距離で止まれる速さ (台形の速度パターン)
            v = fminf(v + LINEAR_ACCEL * DT, CRUISE_SPEED);
            v = fminf(v, sqrtf(2.0f * LINEAR_ACCEL * remain));
            v = fmaxf(v, MIN_SPEED);
            // 目標の角を向く。終わり際は向きの計算が不安定なので辺の向きに合わせる
            float heading = (remain > 0.05f) ? atan2f(ty - p.y, tx - p.x) : atan2f(uy, ux);
            float err = wrap_pi(heading - p.theta);
            drive_set(v, clampf(HEADING_KP * err, -TURN_RATE, TURN_RATE));
            break;
        }
        case ST_TURN: {
            // 目標の向きは辺ごとに ±90° ずつ (theta は連続値なので折り返さない)
            float target = (side + 1) * TURN_SIGN * (float)M_PI / 2.0f;
            float err = target - p.theta;
            if (fabsf(err) < TURN_TOLERANCE_RAD) {
                drive_stop();
                w = 0.0f;
                ESP_LOGI(TAG, "旋回 %d 終了: θ=%.1f°", side, DEG(p.theta));
                side++;
                if (side % 4 == 0) {
                    ESP_LOGI(TAG, "%d 周目終了: x=%.3f y=%.3f (スタートからのずれ %.1f mm)", side / 4,
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
            ESP_LOGI(TAG, "x=%.3f y=%.3f θ=%.1f°", p.x, p.y, DEG(p.theta));
        }
    }

    drive_stop();
    pose_t p = drive_pose();
    ESP_LOGI(TAG, "停止: x=%.3f y=%.3f θ=%.1f°", p.x, p.y, DEG(p.theta));
    while (button_pressed()) {   // 押したままだと次のスタートの待ちを抜けてしまうので、離すまで待つ
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    drive_init();
    drive_stop();
    button_init();
    if (AUTO_START_MS > 0) {
        ESP_LOGI(TAG, "%d ms 後にスタートします", AUTO_START_MS);
        vTaskDelay(pdMS_TO_TICKS(AUTO_START_MS));
    } else {
        wait_start_button();
    }
    while (true) {
        if (ROVER_CHECK_MODE) {
            check_wheels();
        } else {
            odom_reset();
            run_square();
        }
        wait_start_button();
    }
}
