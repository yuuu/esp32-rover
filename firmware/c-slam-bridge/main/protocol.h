#pragma once

// UDP packets between the rover and the host (host/ruby-slam/lib/rover_slam/protocol.rb is the
// other side: keep the two the same). Little-endian, packed, each starting with a header.
//
// The rover listens on CONFIG_ROVER_UDP_PORT. It sends nothing until a host sends it a packet,
// then streams to the address the last one came from.
#include <stdint.h>

#define PROTO_MAGIC 0x52   // 'R'

enum {
    // host -> rover
    PKT_CMD = 'C',     // cmd_payload_t: velocity command. Send every 100 ms; 0, 0 to stop
    // rover -> host
    PKT_ODOM = 'O',    // odom_payload_t, every ODOM_PERIOD_MS
    PKT_LIDAR = 'L',   // the bytes the X2 sent since the last one, as they came (up to LIDAR_CHUNK)
};

typedef struct __attribute__((packed)) {
    uint8_t magic;   // PROTO_MAGIC
    uint8_t type;    // PKT_*
    uint16_t seq;    // per type, wraps
    uint32_t t_ms;   // the sender's clock (the rover's: ms since boot, when the data was taken)
} pkt_header_t;

typedef struct __attribute__((packed)) {
    float v;   // [m/s]
    float w;   // [rad/s], counterclockwise positive
} cmd_payload_t;

typedef struct __attribute__((packed)) {
    float x, y, theta;   // pose from wheel odometry [m, m, rad] (theta continuous)
    float v, w;          // measured velocities [m/s, rad/s]
    uint8_t flags;       // ODOM_FLAG_*
} odom_payload_t;

#define ODOM_FLAG_STOPPED 0x01   // stopped by the BOOT button or the command watchdog
#define ODOM_FLAG_ESTOP 0x02     // stopped by the BOOT button (press again to release)
