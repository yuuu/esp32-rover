// Counts the A/B encoder phases with PCNT (quadrature x4)
#include "encoder.h"

#include "config.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

// The PCNT counter is 16-bit. Each time it reaches this value, it is accumulated internally (accum_count)
#define PCNT_LIMIT 10000

static pcnt_unit_handle_t s_units[WHEEL_COUNT];
static bool s_invert[WHEEL_COUNT];

static pcnt_unit_handle_t new_quadrature_unit(gpio_num_t pin_a, gpio_num_t pin_b)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -PCNT_LIMIT,
        .high_limit = PCNT_LIMIT,
        .flags.accum_count = true,
    };
    pcnt_unit_handle_t unit;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));

    // Two channels count both rising and falling edges of A and B (x4)
    pcnt_chan_config_t cfg_a = {.edge_gpio_num = pin_a, .level_gpio_num = pin_b};
    pcnt_chan_config_t cfg_b = {.edge_gpio_num = pin_b, .level_gpio_num = pin_a};
    pcnt_channel_handle_t ch_a, ch_b;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &cfg_a, &ch_a));
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &cfg_b, &ch_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Accumulated counting needs watch points at both limits
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, -PCNT_LIMIT));

    // Pull up so open-collector encoder outputs can be read too
    gpio_pullup_en(pin_a);
    gpio_pullup_en(pin_b);

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

void encoder_init(void)
{
    pcnt_unit_handle_t a = new_quadrature_unit(PIN_ENC_A_A, PIN_ENC_A_B);
    pcnt_unit_handle_t b = new_quadrature_unit(PIN_ENC_B_A, PIN_ENC_B_B);
    s_units[WHEEL_L] = LEFT_IS_A ? a : b;
    s_units[WHEEL_R] = LEFT_IS_A ? b : a;
    s_invert[WHEEL_L] = ENC_L_INVERT;
    s_invert[WHEEL_R] = ENC_R_INVERT;
}

int32_t encoder_get(wheel_t w)
{
    int count = 0;
    pcnt_unit_get_count(s_units[w], &count);
    return s_invert[w] ? -count : count;
}
