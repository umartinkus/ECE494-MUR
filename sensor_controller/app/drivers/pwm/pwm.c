#include <stdio.h>
#include "pwm.h"

void thruster_config(){
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // 2 ms period per thruster spec
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // prepare and apply PWM channel config
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = THRUSTER_1,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
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
