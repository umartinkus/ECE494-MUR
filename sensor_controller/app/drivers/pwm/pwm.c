#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "pwm.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "hal/ledc_types.h"

int thruster_gpio_array[6] = {15, 2, 0, 4, 16, 17};

void init_pwm_array(ledc_channel_config_t* pwm_arr, int num_channels) {
    for (int i = 0; i < num_channels; i++) {
    pwm_arr[i] = (ledc_channel_config_t) {
      .gpio_num = thruster_gpio_array[i],
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = i,
      .timer_sel = LEDC_TIMER_0,
      .intr_type = LEDC_INTR_DISABLE,
      .duty = 0,
      .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_arr[i]));
  }
}

void init_timer(ledc_timer_config_t* ledc_timer){
    // Prepare and then apply the LEDC PWM timer configuration
    *ledc_timer = (ledc_timer_config_t) {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // 2 ms period per thruster spec
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(ledc_timer));
}

float map_ctrl_value(uint8_t ctrl_value, float deadzone) {
    // returns a value between -1 and 1 based on control input
    float mapped = (float)ctrl_value * (2.0f / 255.0f) - 1.0f;

    // apply deadzone
    if (fabsf(mapped) < deadzone) {
        return 0.0f;
    } else {
        return mapped;
    }
}

uint32_t mapped_to_duty(float ctrl_value) {
    if (ctrl_value > 1) {
        ctrl_value = 1;
    } else if (ctrl_value < -1) {
        ctrl_value = -1;
    }

    float us = 500.0f * ctrl_value + 1500.0f;
    return (uint32_t)us * LEDC_MAX_DUTY / LEDC_PERIOD_US;
    
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
