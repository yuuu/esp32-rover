#pragma once

// The UDP link to the host (protocol.h)
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void link_init(void);
// Send one packet of the given type to the host; dropped while no host has been heard from
void link_send(uint8_t type, uint32_t t_ms, const void *payload, size_t len);
// The latest velocity command. False when none came within CMD_TIMEOUT_MS
bool link_command(float *v, float *w);
