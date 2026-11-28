#pragma once

#include <stdint.h>

#include "motor.h"

void encoder_init(void);
// Accumulated count since boot (quadrature x4). Increases when the wheel turns forward
int32_t encoder_get(wheel_t w);
