#include "button.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rover_config.h"

void button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_START_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&cfg);
}

bool button_pressed(void)
{
    return gpio_get_level(PIN_START_BUTTON) == 0;
}

void button_wait_release(void)
{
    while (button_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void button_wait_click(void)
{
    while (!button_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    button_wait_release();
    vTaskDelay(pdMS_TO_TICKS(1000));
}
