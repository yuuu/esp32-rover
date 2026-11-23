#pragma once

#include "driver/gpio.h"

// ================================================================ ピン
// kicad/main の配線 (XIAO ESP32-S3 の Dn → GPIO)。
// モータドライバ TC78H653FTG は MODE=GND (IN/IN モード)、STBY は 3V3 に固定
#define PIN_MOTOR_A_IN1 GPIO_NUM_1   // D0 → IN1 → J1 pin1 (OUT1)
#define PIN_MOTOR_A_IN2 GPIO_NUM_2   // D1 → IN2 → J1 pin2 (OUT2)
#define PIN_MOTOR_B_IN1 GPIO_NUM_3   // D2 → IN3 → J3 pin1 (OUT3)
#define PIN_MOTOR_B_IN2 GPIO_NUM_4   // D3 → IN4 → J3 pin2 (OUT4)
// エンコーダの A/B 相。電源は J1/J3 の pin3 (3V3)、GND は J2/J4 の pin3
#define PIN_ENC_A_A GPIO_NUM_5       // D4 → J2 pin1
#define PIN_ENC_A_B GPIO_NUM_6       // D5 → J2 pin2
#define PIN_ENC_B_A GPIO_NUM_7       // D8 → J4 pin1
#define PIN_ENC_B_B GPIO_NUM_8       // D9 → J4 pin2
#define PIN_START_BUTTON GPIO_NUM_0  // XIAO の BOOT ボタン (押すと LOW)

// ================================================================ 左右の割り当てと向き
// ROVER_CHECK_MODE を 1 にして書き込むと確認できる。下の値は実機で確認済み
#define LEFT_IS_A 0        // 1: 左が モータA(J1)/エンコーダA(J2)、0: 左が B(J3/J4)
#define MOTOR_L_INVERT 1   // 正のデューティで車輪が後ろへ回るなら 1
#define MOTOR_R_INVERT 0   // 左右のモータは向かい合わせなので、片方は逆になる
#define ENC_L_INVERT 0     // 車輪が前へ回ったときにカウントが減るなら 1
#define ENC_R_INVERT 1

// 1 にすると走行せず、左右の車輪を順に少し回してエンコーダの値を表示する
#ifndef ROVER_CHECK_MODE
#define ROVER_CHECK_MODE 0
#endif

// ================================================================ 機体
#define WHEEL_DIAMETER_M 0.060f   // タイヤ FS90R-W Φ60
#define TRACK_WIDTH_M 0.092f      // 車輪中心面 Y=±46
// MG12A06E-060310-50: エンコーダ 7PPR × 減速比 50 × 4逓倍
#define ENC_COUNTS_PER_REV (7 * 50 * 4)

// ================================================================ 制御
#define CONTROL_PERIOD_MS 10
// デューティ 1.0 で出る車輪の速さの目安 (フィードフォワード用、実機で合わせる)
#define WHEEL_SPEED_AT_FULL_DUTY 0.60f   // [m/s]
#define WHEEL_KP 1.5f                    // 速度誤差 [m/s] → デューティ
#define WHEEL_KI 6.0f
#define WHEEL_SPEED_LPF 0.3f             // 速度の測定値のローパス (1 で無効)

// ================================================================ 走行 (正方形)
#define SQUARE_SIDE_M 0.50f
#define SQUARE_TURN_LEFT 1        // 1: 左回り (反時計回り)、0: 右回り
#define SQUARE_LAPS 0             // 周回数。0 で止めるまで回り続ける (BOOT ボタンで止まる)
#define AUTO_START_MS 3000        // 電源を入れてから走り出すまで。0 なら BOOT ボタンを待つ
#define CRUISE_SPEED 0.25f        // 直進の速さ [m/s] (0.15 で正方形を確認済み)
#define LINEAR_ACCEL 0.50f        // [m/s^2]
#define MIN_SPEED 0.03f           // 目標の直前でもこれ以上で進む
#define HEADING_KP 4.0f           // 方位の誤差 [rad] → 角速度 [rad/s]
#define TURN_RATE 2.5f            // その場旋回の角速度 [rad/s]
#define ANGULAR_ACCEL 6.0f        // [rad/s^2]
#define MIN_TURN_RATE 0.3f
#define TURN_TOLERANCE_RAD 0.01f  // 約0.6°
#define PAUSE_MS 300              // 直進と旋回の間で止まる時間
