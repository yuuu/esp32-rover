#pragma once

#include <stdint.h>

#include "motor.h"

void encoder_init(void);
// 起動からの累積カウント (4逓倍)。車輪が前へ回ると増える
int32_t encoder_get(wheel_t w);
