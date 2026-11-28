#pragma once

// Turn each wheel forward briefly and print how its encoder count changed, to check LEFT_IS_A and
// the *_INVERT flags in rover_config.h after changing the wiring. Call after drive_init().
void rover_check_wheels(void);
