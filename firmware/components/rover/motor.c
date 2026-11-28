// Motor drive for the TC78H653FTG (IN/IN mode)
//
// One input is held H and PWM goes to the other, switching between drive and brake.
// Compared with drive / coast (one input held L), duty maps to speed more linearly, even at low speed.
//   Forward: IN1=H, IN2=PWM (brake while H) -> duty d is the fraction of IN2 that is L
//   Reverse: IN1=PWM, IN2=H
//   Stop:    IN1=IN2=H (brake)
#include "motor.h"

#include <math.h>

#include "rover_config.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define PWM_FREQ_HZ 20000
#define PWM_RES LEDC_TIMER_10_BIT
#define PWM_FULL (1 << 10)   // LEDC reaches 100% at duty 2^bit

typedef struct {
    ledc_channel_t ch_in1;
    ledc_channel_t ch_in2;
    bool invert;
} motor_t;

static motor_t s_motors[WHEEL_COUNT];

static void add_channel(ledc_channel_t ch, gpio_num_t pin)
{
    ledc_channel_config_t cfg = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ch,
        .timer_sel = LEDC_TIMER_0,
        .duty = PWM_FULL,   // brake at startup
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

static void set_duty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

void motor_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    add_channel(LEDC_CHANNEL_0, PIN_MOTOR_A_IN1);
    add_channel(LEDC_CHANNEL_1, PIN_MOTOR_A_IN2);
    add_channel(LEDC_CHANNEL_2, PIN_MOTOR_B_IN1);
    add_channel(LEDC_CHANNEL_3, PIN_MOTOR_B_IN2);

    motor_t a = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, false};
    motor_t b = {LEDC_CHANNEL_2, LEDC_CHANNEL_3, false};
    s_motors[WHEEL_L] = LEFT_IS_A ? a : b;
    s_motors[WHEEL_R] = LEFT_IS_A ? b : a;
    s_motors[WHEEL_L].invert = MOTOR_L_INVERT;
    s_motors[WHEEL_R].invert = MOTOR_R_INVERT;
}

void motor_set(wheel_t w, float duty)
{
    const motor_t *m = &s_motors[w];
    if (m->invert) {
        duty = -duty;
    }
    duty = fmaxf(-1.0f, fminf(1.0f, duty));
    uint32_t off = (uint32_t)lroundf((1.0f - fabsf(duty)) * PWM_FULL);
    if (duty >= 0.0f) {
        set_duty(m->ch_in1, PWM_FULL);
        set_duty(m->ch_in2, off);
    } else {
        set_duty(m->ch_in1, off);
        set_duty(m->ch_in2, PWM_FULL);
    }
}
