#include "lidar.h"

#include "bridge_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "link.h"
#include "protocol.h"
#include "rover_config.h"

#define LIDAR_UART UART_NUM_1

static const char *TAG = "lidar";

static void lidar_task(void *arg)
{
    static uint8_t buf[LIDAR_CHUNK];
    uint32_t total = 0;
    int64_t last_log = esp_timer_get_time();
    while (true) {
        // Returns when the buffer is full or LIDAR_FLUSH_MS passed, so latency stays bounded
        int n = uart_read_bytes(LIDAR_UART, buf, sizeof(buf), pdMS_TO_TICKS(LIDAR_FLUSH_MS));
        if (n > 0) {
            link_send(PKT_LIDAR, (uint32_t)(esp_timer_get_time() / 1000), buf, n);
            total += n;
        }
        int64_t now = esp_timer_get_time();
        if (now - last_log > 10 * 1000000LL) {
            ESP_LOGI(TAG, "%lu bytes/s from the X2", (unsigned long)(total / 10));
            total = 0;
            last_log = now;
        }
    }
}

void lidar_start(void)
{
    uart_config_t cfg = {
        .baud_rate = LIDAR_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, 4096, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, UART_PIN_NO_CHANGE, PIN_LIDAR_RX, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Reading the X2 on GPIO%d", PIN_LIDAR_RX);
    xTaskCreate(lidar_task, "lidar", 4096, NULL, 5, NULL);
}
