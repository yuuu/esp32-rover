#pragma once

#include <stdbool.h>

// The XIAO BOOT button (PIN_START_BUTTON), used to start and stop runs
void button_init(void);
bool button_pressed(void);
// Wait for a press and release, then one more second so the hand is off the robot
void button_wait_click(void);
// Wait until the button is released (a held button would otherwise count as the next click)
void button_wait_release(void);
