#include <stdint.h>
#include "pwm.h"
#include "common_types.h"
#include "driver/ledc.h"

int thruster_gpio_array[6] = {16, 17, 21, 25, 26, 27};

error_code_t init_pwm_array(ledc_channel_config_t *pwm_arr, int num_channels)
{
    if (pwm_arr == NULL || num_channels < 1 || num_channels > 6) {
        return STATUS_CONFIG_ERR;
    }

    error_code_t pwm_err = STATUS_OK;
    for (int i = 0; i < num_channels; i++) {
        pwm_arr[i] = (ledc_channel_config_t) {
            .gpio_num = thruster_gpio_array[i],
            .speed_mode = LEDC_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .duty = 0,
            .hpoint = 0
        };
        if (ledc_channel_config(&pwm_arr[i]) != ESP_OK) {
            pwm_err = STATUS_ERROR;
        }
    }
    return pwm_err;
}

error_code_t init_timer(ledc_timer_config_t *ledc_timer)
{
    if (ledc_timer == NULL) {
        return STATUS_CONFIG_ERR;
    }

    *ledc_timer = (ledc_timer_config_t) {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(ledc_timer) != ESP_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

uint32_t pwm_us_to_duty(uint32_t pulse_us)
{
    if (pulse_us > LEDC_PERIOD_US) {
        pulse_us = LEDC_PERIOD_US;
    }

    return (pulse_us * LEDC_MAX_DUTY) / LEDC_PERIOD_US;
}

esp_err_t thruster_set_pulse_us(ledc_channel_t channel, uint32_t pulse_us)
{
    uint32_t duty = pwm_us_to_duty(pulse_us);
    esp_err_t err = ledc_set_duty(LEDC_MODE, channel, duty);
    if (err != ESP_OK) {
        return err;
    }

    return ledc_update_duty(LEDC_MODE, channel);
}
