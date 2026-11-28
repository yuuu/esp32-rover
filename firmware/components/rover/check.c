#include "check.h"

#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"
#include "rover_config.h"

static const char *TAG = "check";

void rover_check_wheels(void)
{
    static const char *names[] = {"left", "right"};
    for (int w = 0; w < WHEEL_COUNT; w++) {
        int32_t before = encoder_get(w);
        ESP_LOGI(TAG, "Turning the %s wheel forward (1 s)", names[w]);
        motor_set(w, 0.3f);
        vTaskDelay(pdMS_TO_TICKS(1000));
        motor_set(w, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(300));
        int32_t delta = encoder_get(w) - before;
        ESP_LOGI(TAG, "%s: count %+ld (%.2f rev)", names[w], (long)delta,
                 (float)delta / ENC_COUNTS_PER_REV);
    }
    ESP_LOGI(TAG, "Check: did the named wheel turn (else LEFT_IS_A), did it turn forward "
                  "(else MOTOR_*_INVERT), is the count positive (else ENC_*_INVERT)");
}
