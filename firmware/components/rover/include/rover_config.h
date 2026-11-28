#pragma once

// Hardware of the rover shared by every C project under firmware/: pins, wheel mapping and
// direction, dimensions and wheel speed control. What a project does with the rover (its driving
// parameters) goes in that project's own header.

#include "driver/gpio.h"

// ================================================================ Pins
// Wiring of pcb/main (XIAO ESP32-S3 Dn -> GPIO).
// The TC78H653FTG motor driver runs with MODE=GND (IN/IN mode) and STBY tied to 3V3
#define PIN_MOTOR_A_IN1 GPIO_NUM_1   // D0 -> IN1 -> J1 pin1 (OUT1)
#define PIN_MOTOR_A_IN2 GPIO_NUM_2   // D1 -> IN2 -> J1 pin2 (OUT2)
#define PIN_MOTOR_B_IN1 GPIO_NUM_3   // D2 -> IN3 -> J3 pin1 (OUT3)
#define PIN_MOTOR_B_IN2 GPIO_NUM_4   // D3 -> IN4 -> J3 pin2 (OUT4)
// Encoder A/B phases. Power is J1/J3 pin3 (3V3), GND is J2/J4 pin3
#define PIN_ENC_A_A GPIO_NUM_5       // D4 -> J2 pin1
#define PIN_ENC_A_B GPIO_NUM_6       // D5 -> J2 pin2
#define PIN_ENC_B_A GPIO_NUM_7       // D8 -> J4 pin1
#define PIN_ENC_B_B GPIO_NUM_8       // D9 -> J4 pin2
#define PIN_START_BUTTON GPIO_NUM_0  // XIAO BOOT button (LOW when pressed)
// YDLIDAR X2 (J5, PH 5P). The X2 only transmits, at 115200 baud 8N1
#define PIN_LIDAR_RX GPIO_NUM_44     // D7 <- J5 pin2 (LiDAR Tx)
#define PIN_LIDAR_MCTR GPIO_NUM_9    // D10 -> J5 pin5 (M_CTR; left as an input, the X2's default speed)

// ================================================================ Left/right mapping and direction
// Check these with rover_check_wheels() (c-square built with ROVER_CHECK_MODE=1).
// The values below are verified on the robot
#define LEFT_IS_A 0        // 1: left is motor A (J1) / encoder A (J2), 0: left is B (J3/J4)
#define MOTOR_L_INVERT 1   // 1 if a positive duty turns the wheel backward
#define MOTOR_R_INVERT 0   // The motors face each other, so one side is reversed
#define ENC_L_INVERT 0     // 1 if the count decreases when the wheel turns forward
#define ENC_R_INVERT 1

// ================================================================ Robot
#define WHEEL_DIAMETER_M 0.060f   // FS90R-W tire, 60mm
#define TRACK_WIDTH_M 0.092f      // Wheel center planes at Y=+-46mm
// MG12A06E-060310-50: encoder 7PPR x gear ratio 50 x 4 (quadrature)
#define ENC_COUNTS_PER_REV (7 * 50 * 4)

// ================================================================ Control
#define CONTROL_PERIOD_MS 10
// Approximate wheel speed at duty 1.0 (for feedforward; tune on the robot)
#define WHEEL_SPEED_AT_FULL_DUTY 0.60f   // [m/s]
#define WHEEL_KP 1.5f                    // speed error [m/s] -> duty
#define WHEEL_KI 6.0f
#define WHEEL_SPEED_LPF 0.3f             // low-pass filter on measured speed (1 disables it)
