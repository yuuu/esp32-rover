#pragma once

typedef enum { WHEEL_L = 0, WHEEL_R = 1, WHEEL_COUNT } wheel_t;

void motor_init(void);
// duty: -1.0 (後退) 〜 +1.0 (前進)。0 でブレーキ
void motor_set(wheel_t w, float duty);
