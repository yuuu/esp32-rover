#pragma once

typedef enum { WHEEL_L = 0, WHEEL_R = 1, WHEEL_COUNT } wheel_t;

void motor_init(void);
// duty: -1.0 (reverse) to +1.0 (forward). 0 brakes
void motor_set(wheel_t w, float duty);
