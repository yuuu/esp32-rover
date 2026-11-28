#include "link.h"

#include <string.h>

#include "bridge_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "protocol.h"

static const char *TAG = "link";
static int s_sock = -1;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static struct sockaddr_in s_host;   // guarded by s_lock
static bool s_has_host;
static float s_cmd_v, s_cmd_w;
static int64_t s_cmd_us;
static uint16_t s_seq[256];         // per packet type; only the sending tasks touch their own types

static void rx_task(void *arg)
{
    uint8_t buf[64];
    while (true) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(s_sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        if (n < (int)sizeof(pkt_header_t)) {
            continue;
        }
        pkt_header_t h;
        memcpy(&h, buf, sizeof(h));
        if (h.magic != PROTO_MAGIC) {
            continue;
        }
        bool new_host;
        taskENTER_CRITICAL(&s_lock);
        new_host = !s_has_host || s_host.sin_addr.s_addr != from.sin_addr.s_addr ||
                   s_host.sin_port != from.sin_port;
        s_host = from;
        s_has_host = true;
        if (h.type == PKT_CMD && n >= (int)(sizeof(h) + sizeof(cmd_payload_t))) {
            cmd_payload_t c;
            memcpy(&c, buf + sizeof(h), sizeof(c));
            s_cmd_v = c.v;
            s_cmd_w = c.w;
            s_cmd_us = esp_timer_get_time();
        }
        taskEXIT_CRITICAL(&s_lock);
        if (new_host) {
            char addr[16];
            inet_ntoa_r(from.sin_addr, addr, sizeof(addr));
            ESP_LOGI(TAG, "Streaming to %s:%d", addr, ntohs(from.sin_port));
        }
    }
}

void link_init(void)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_ROVER_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    ESP_ERROR_CHECK(bind(s_sock, (struct sockaddr *)&addr, sizeof(addr)) == 0 ? ESP_OK : ESP_FAIL);
    ESP_LOGI(TAG, "Listening on UDP port %d", CONFIG_ROVER_UDP_PORT);
    xTaskCreate(rx_task, "link_rx", 4096, NULL, 6, NULL);
}

void link_send(uint8_t type, uint32_t t_ms, const void *payload, size_t len)
{
    struct sockaddr_in host;
    taskENTER_CRITICAL(&s_lock);
    bool has_host = s_has_host;
    host = s_host;
    taskEXIT_CRITICAL(&s_lock);
    if (!has_host) {
        return;
    }
    uint8_t buf[sizeof(pkt_header_t) + LIDAR_CHUNK];
    if (len > LIDAR_CHUNK) {
        len = LIDAR_CHUNK;
    }
    pkt_header_t h = {.magic = PROTO_MAGIC, .type = type, .seq = s_seq[type]++, .t_ms = t_ms};
    memcpy(buf, &h, sizeof(h));
    memcpy(buf + sizeof(h), payload, len);
    // A full send buffer drops the packet; the next one carries newer data anyway
    sendto(s_sock, buf, sizeof(h) + len, MSG_DONTWAIT, (struct sockaddr *)&host, sizeof(host));
}

bool link_command(float *v, float *w)
{
    taskENTER_CRITICAL(&s_lock);
    bool fresh = s_cmd_us > 0 && esp_timer_get_time() - s_cmd_us < CMD_TIMEOUT_MS * 1000LL;
    *v = s_cmd_v;
    *w = s_cmd_w;
    taskEXIT_CRITICAL(&s_lock);
    return fresh;
}
